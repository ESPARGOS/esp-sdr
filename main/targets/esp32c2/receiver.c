/* ESP32-C2 burst SDR over UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"

#include "burst_serial.h"
#include "burst_gpio.h"
#include "burst_version.h"
#include "spectrum.h"
#include "rx_recalibration.h"
#include "rx_tuning.h"
#include "rx_lo.h"

/* Pinned C2 librftest adctrig uses 32 KiB at 0x3fcc0000 and
 * SRAM usage 2 in SENSITIVE 0x600c1018. heap_guard.c also excludes the
 * rest of this 128 KiB bank: CPU/DMA traffic there corrupts RF dumps
 * and unrelated memory. Only quiescent FFT storage lives above the buffer. */
SOC_RESERVE_MEMORY_REGION(0x3fcc0000, 0x3fcc8000, c2_rf_dump);
#define IQ_WORDS 8190u
#define IQ_BUFFER ((uint32_t *)0x3fcc0000)
#define SRAM_OWNER_REG 0x600c1018u
extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned,unsigned);
extern void phy_set_freq(unsigned,int);
extern unsigned rom_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void rom_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
static int rx_filter=-1;
static unsigned rx_filter_saved[2];
/* BBTOP I/Q CBW20 capacitor codes. Preserve calibration and upper bits. */
static void rx_filter_apply(void) {
    for(unsigned j=0;j<2;j++) {
        rx_filter_saved[j]=rom_chip_i2c_readReg(0x67,1,4+j);
        if(rx_filter>=0)rom_chip_i2c_writeReg(0x67,1,4+j,(rx_filter_saved[j]&~63u)|(unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if(rx_filter>=0)for(unsigned j=0;j<2;j++)rom_chip_i2c_writeReg(0x67,1,4+j,rx_filter_saved[j]);
}
static unsigned frequency_mhz=2412;
static bool rx_ready;
#define C2_FREQ_MIN RX_FREQ_MIN
#define C2_FREQ_MAX RX_FREQ_MAX
static bool frequency_valid(unsigned mhz) {
    return mhz>=C2_FREQ_MIN && mhz<=C2_FREQ_MAX;
}
static void tune_rx(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    bool channel=(mhz>=2412 && mhz<=2472 && (mhz-2412)%5==0) || mhz==2484;
    /* Calibrate using a real Wi-Fi channel, then program exact PLL MHz.
     * The channel API otherwise rounds off-grid frequencies. This is an
     * attempt range; PLL lock and reception are not guaranteed throughout. */
    rx_lo_plan_t plan=rx_lo_plan(mhz);
    rx_lo_select(false);
    set_chanfreq(channel?mhz:2412,0);
    if(!channel)phy_set_freq(plan.mhz,plan.offset_khz);
}
#define send_bytes burst_serial_send
static void reply(const char *s) { (void)send_bytes(s,strlen(s)); }
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if(rx_ready && !rx_recalibration_stale())return;
    tune_rx(frequency_mhz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    rx_ready=true;
}

static size_t packed_size(unsigned n) { return (n*20u+7u)/8u; }
/* Two complete IQ10 samples occupy five bytes; an odd tail occupies three. */
static void pack_iq(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j+=2,p+=5) {
        uint32_t a=IQ_BUFFER[j]&0xfffffu;
        uint32_t b=j+1<n?IQ_BUFFER[j+1]&0xfffffu:0;
        p[0]=a;p[1]=a>>8;p[2]=(a>>16)|(b<<4);
        if(j+1<n){p[3]=b>>4;p[4]=b>>12;}
    }
}

/* IQ8 is signed two's complement I then Q. Retain each IQ10 field's
 * upper eight bits (arithmetic truncation). */
static void pack_iq8(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j++) {
        uint32_t w=IQ_BUFFER[j];p[2*j]=(w>>2)&255;p[2*j+1]=(w>>12)&255;
    }
}

static size_t wire_size(unsigned n,unsigned format) {
    return format==16?n*2:format==20?packed_size(n):n*4;
}
static portMUX_TYPE capture_mux=portMUX_INITIALIZER_UNLOCKED;
static bool acquire_iq(unsigned n,unsigned divider,unsigned *capture_us) {
    if(divider!=0 && divider!=1 && divider!=6){reply("ERR rate\n");return false;}
    prepare_rx();

    for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
    rx_filter_apply();
    /* Prevent task/interrupt interleaving during the bank handoff and
     * bounded snapshot (at most about 512 us at 16 MS/s). */
    taskENTER_CRITICAL(&capture_mux);
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    uint32_t clock=REG_READ(0x60026014);
    uint32_t trigger=REG_READ(0x600460b4);
    uint32_t dump_mode=REG_READ(0x6004bc08);
    uint32_t lanes=REG_READ(0x6004bc14);
    REG_WRITE(0x60026014,clock|(1u<<29));
    REG_WRITE(0x600460b4,trigger&~1u);
    REG_WRITE(0x6004bc08,dump_mode|(15u<<15));
    unsigned ticks_per_us=esp_rom_get_cpu_ticks_per_us();
    uint32_t start=esp_cpu_get_cycle_count();
    REG_WRITE(0x6004bc04,0);
    REG_WRITE(0x6004bc14,(1u<<6)|(2u<<12)|(3u<<18));
    REG_WRITE(SRAM_OWNER_REG,(owner&~7u)|2u);
    __asm__ volatile("fence rw,rw" ::: "memory");
    unsigned dump_clock=divider==6?2:divider;
    uint32_t ctrl=0x80000000u|(dump_clock<<15)|n;
    REG_WRITE(0x6004bc04,ctrl);
    REG_WRITE(0x6004bc04,ctrl|(1u<<19));
    REG_WRITE(0x6004bc04,ctrl);
    while(!(REG_READ(0x6004bc04)&(1u<<18)) && (uint32_t)(esp_cpu_get_cycle_count()-start)<20000u*ticks_per_us){}
    bool done=(REG_READ(0x6004bc04)&(1u<<18))!=0;
    uint32_t elapsed=(esp_cpu_get_cycle_count()-start)/ticks_per_us;
    /* Disable before changing the dump-clock selection, as in vendor adctrig. */
    REG_WRITE(0x6004bc04,ctrl&~0x80000000u);
    uint32_t stopped=esp_cpu_get_cycle_count();
    while((uint32_t)(esp_cpu_get_cycle_count()-stopped)<5u*ticks_per_us){}
    __asm__ volatile("fence rw,rw" ::: "memory");
    REG_WRITE(SRAM_OWNER_REG,owner);
    REG_WRITE(0x6004bc14,lanes);
    REG_WRITE(0x6004bc08,dump_mode);
    REG_WRITE(0x600460b4,trigger);
    REG_WRITE(0x60026014,clock);
    __asm__ volatile("fence rw,rw" ::: "memory");
    taskEXIT_CRITICAL(&capture_mux);
    rx_filter_restore();
    if(!done){reply("ERR capture_timeout\n");return false;}
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au){reply("ERR capture_incomplete\n");return false;}
    }
    *capture_us = elapsed;
    return true;
}

static bool spectrum_acquire(unsigned n, unsigned rate, const uint32_t **data, unsigned *elapsed) {
    bool ok = acquire_iq(n, rate, elapsed);
    *data = IQ_BUFFER;
    return ok;
}


static bool capture(unsigned n,unsigned divider,unsigned format) {
    unsigned elapsed;
    if (!acquire_iq(n, divider, &elapsed)) return false;
    size_t bytes=wire_size(n,format);
    if(format==16)pack_iq8(n);else if(format==20)pack_iq(n);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    char h[96];
    snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIu32 "\n",n,crc,(uint32_t)elapsed);
    return send_bytes(h,strlen(h)) && send_bytes(IQ_BUFFER,bytes);
}

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
    if(!strcmp(line,"TRANSPORT?")) {
        char answer[64];
        snprintf(answer,sizeof(answer),"TRANSPORT %s %u\n",
                 burst_serial_port()==BURST_SERIAL_UART?"UART":"USB",burst_serial_baud());
        reply(answer);return;
    }
    if(limits_command(line))return;
    if(gain_command(line))return;
    unsigned n,rate,crc,repeats;char extra;uint64_t nonce;
    bool iq8=false;
    if(!strncmp(line,"CAP16 ",6)){memcpy(line,"CAP20",5);iq8=true;}
    if(sscanf(line,"SYNC %" SCNu64 " %c",&nonce,&extra)==1) {
        char answer[48];snprintf(answer,sizeof(answer),"SYNC %" PRIu64 "\n",nonce);reply(answer);
    }
    else if(sscanf(line,"RXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
            n>=256 && n<=IQ_WORDS && rate<=6 && repeats>0 && repeats<=1000 && (crc==16 || crc==20)) {
        /* Format 16 = IQ8, 20 = IQ10 packed. Each frame
         * is a separate capture, with RF gaps during UART transfer. */
        bool ok=true;
        for(unsigned j=0;j<repeats && ok;j++){ok=capture(n,rate,crc);vTaskDelay(1);}
        if(ok)reply("END\n");
    }
    else if(!strcmp(line,"CAPS")) {
        reply("CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS SERIALLEASE "
              "TUNEEXT LPFANA RX40 RX16 GAIN HWAGC IQ8\n");
    }
    else if(sscanf(line,"BANDWIDTH %u %c",&n,&extra)==1 &&
            (!n || (n>=RX_BANDWIDTH_MIN && n<=RX_BANDWIDTH_MAX))) {
        rx_filter=rx_bandwidth_dcap(n);reply("OK\n");
    }
    else if(!strcmp(line,"LPF AUTO")){rx_filter=-1;reply("OK\n");}
    else if(sscanf(line,"LPF %u %c",&n,&extra)==1 && n<=63){rx_filter=n;reply("OK\n");}
    else if(!strcmp(line,"LPF?")) {
        char answer[80];snprintf(answer,sizeof(answer),"LPF %d %u %u\n",rx_filter,
            rom_chip_i2c_readReg(0x67,1,4)&63,rom_chip_i2c_readReg(0x67,1,5)&63);reply(answer);
    }
    else if(!strcmp(line,"RANGE?")) {
        char answer[64];snprintf(answer,sizeof(answer),"RANGE %u %u 1\n",C2_FREQ_MIN,C2_FREQ_MAX);reply(answer);
    }
    else if(!strcmp(line,"INFO")) reply("C2SDR 6 burst 8190\n");
    else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 && frequency_valid(n)) {
        frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
    } else if(((!strncmp(line,"CAP ",4) && sscanf(line,"CAP %u %u %c",&n,&rate,&extra)==2) ||
               (!strncmp(line,"CAP20 ",6) && sscanf(line,"CAP20 %u %u %c",&n,&rate,&extra)==2)) &&
               n>=256 && n<=IQ_WORDS && rate<=6) capture(n,rate,!strncmp(line,"CAP20 ",6)? (iq8?16:20):0);
    else reply("ERR command\n");
}

void app_main(void) {
    esp_log_level_set("*",ESP_LOG_NONE);
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    /* SDR does not queue packets; keep Wi-Fi allocations in the lower banks. */
    cfg.static_rx_buf_num=2;cfg.dynamic_rx_buf_num=4;
    cfg.static_tx_buf_num=2;cfg.dynamic_tx_buf_num=2;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*",ESP_LOG_NONE);
    burst_serial_init();
    char line[128];
    int owner=-1;
    int64_t lease_deadline=0;
    for(;;) {
        if(esp_timer_get_time()>=lease_deadline)owner=-1;
        int status=burst_serial_poll_line(line,sizeof(line));
        if(!status){vTaskDelay(1);continue;}
        int port=burst_serial_port();
        if(owner>=0 && owner!=port){reply("ERR busy\n");continue;}
        if(status<0){reply("ERR command_length\n");continue;}
        owner=port;
        if(!strcmp(line,"RELEASE")) {
            reply("OK\n");owner=-1;
        } else {
            handle_command(line);
        }
        /* Ownership covers the entire binary transaction. Silence releases it
         * after five seconds. */
        lease_deadline=esp_timer_get_time()+5000000;
    }
}
