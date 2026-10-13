/* ESP32-S3 burst SDR over native USB Serial/JTAG and UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"

#include "burst_serial.h"
#include "rx_tuning.h"
#if CONFIG_ESP_SDR_LCD_ES3C28P
#include "lcd_waterfall.h"
#endif

/* Vendor S3 adctrig uses this 64 KiB aperture with MAC_DUMP_USAGE=4.
 * Keep both its DRAM and IRAM aliases out of the heap and static sections. */
SOC_RESERVE_MEMORY_REGION(0x3fcd0000, 0x3fce0000, s3_rf_dump);
#define IQ_WORDS 16380u
#define IQ_BUFFER ((uint32_t *)0x3fcd0000)
#define SRAM_OWNER_REG 0x600c101cu
extern void adctrig(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
extern void stop_tx_tone(unsigned);
#define phy_stop_tx_tone stop_tx_tone
extern void rom_pbus_workmode(void);
#define phy_pbus_workmode rom_pbus_workmode
extern void rom_pbus_xpd_rx_on(unsigned);
#define phy_pbus_xpd_rx_on rom_pbus_xpd_rx_on
extern void rom_pbus_xpd_tx_off(void);
#define phy_pbus_xpd_tx_off rom_pbus_xpd_tx_off
extern void rom_set_rxclk_en(unsigned);
#define phy_set_rxclk_en rom_set_rxclk_en
extern void set_chanfreq(unsigned,unsigned);
extern void set_rf_freq_offset(unsigned,unsigned,int);
static void s3_tune(unsigned mhz) {
    bool channel=(mhz>=2412 && mhz<=2472 && (mhz-2412)%5==0)||mhz==2484;
    set_chanfreq(channel?mhz:2412,0);
    if(!channel)set_rf_freq_offset(0,mhz,0); /* 40 MHz crystal; direct PLL MHz. */
}

#define S3_FREQ_MIN RX_FREQ_MIN
#define S3_FREQ_MAX RX_FREQ_MAX
static unsigned frequency_mhz=2412;
static bool rx_ready;
#ifdef S3_RF_PROBE
static unsigned rx_clock=0;
static unsigned rx_source,rx_mode,rx_flag,rx_wide,rx_prep=3,rx_pack,rx_agc;
#else
enum { rx_source=0,rx_mode=0,rx_flag=0,rx_wide=0,rx_prep=3,rx_pack=0,rx_agc=0 };
#endif
extern void force_rx_gain(unsigned,unsigned,unsigned);
static int rx_filter=-1; /* -1 restores the PHY-calibrated automatic mode. */
extern unsigned rom_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void rom_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
/* Apply only around an RX snapshot; restore before any retune. */
static unsigned rx_filter_saved[2];
static void rx_filter_apply(void) {
    for(unsigned j=0;j<2;j++) {
        rx_filter_saved[j]=rom_chip_i2c_readReg(0x67,0,4+j);
        if(rx_filter>=0)rom_chip_i2c_writeReg(0x67,0,4+j,(rx_filter_saved[j]&~63u)|(unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if(rx_filter>=0)for(unsigned j=0;j<2;j++)rom_chip_i2c_writeReg(0x67,0,4+j,rx_filter_saved[j]);
}
/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char *cmd,char *name,int *argc,char **argv) {
    (void)cmd;(void)name;(void)argc;(void)argv;return -1;
}
#define send_bytes burst_serial_send
static void reply(const char *s) { (void)send_bytes(s,strlen(s)); }
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if(rx_ready)return;
    if(rx_prep==1){esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);force_rx_gain(1,55,0);rx_ready=true;return;}
    if(rx_prep==2){esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);rx_ready=true;return;}
    s3_tune(frequency_mhz);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_ready=true;
}
#include "filter_probe.h"

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
#ifdef SAMPLE_RATE_PROBE
/* Volatile, bounded dump-clock/source investigation; excluded from releases. */
static unsigned probe_source,probe_clock,probe_adc=4;
extern void rom_dac_rate_set(unsigned);
static bool probe_capture;
#endif
/* Fill IQ_BUFFER with n raw IQ10 words; false on timeout. */
static bool capture_raw(unsigned n,unsigned divider,uint32_t *elapsed_us) {
    prepare_rx();
    
    for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
    rx_filter_apply();
#ifdef SAMPLE_RATE_PROBE
    unsigned adc_saved=rom_chip_i2c_readReg(0x66,0,4);
    if(probe_adc<4)rom_chip_i2c_writeReg(0x66,0,4,(adc_saved&~12u)|(probe_adc<<2));
#endif
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    int64_t start=esp_timer_get_time();
    REG_WRITE(0x60033d5c,0);
    REG_WRITE(0x60033d90,rx_pack|((rx_pack+1)<<6)|((rx_pack+2)<<12)|((rx_pack+3)<<18)|(rx_agc<<24));
    REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    /* Native dump clocks verified with B210: bit15 halves 80 to 40 MS/s;
     * bit16 selects the 16 MS/s hardware path. No software resampling. */
    uint32_t rate_bits=divider==1?(1u<<15):divider==6?(1u<<16):0;
    uint32_t ctrl=0x80000000u|rate_bits|(rx_wide<<17)|(rx_flag<<16)|(rx_source<<20)|(rx_mode<<28)|n;
#ifdef S3_RF_PROBE
    ctrl|=rx_clock<<15;
#endif
#ifdef SAMPLE_RATE_PROBE
    if(probe_capture)ctrl=(ctrl&~0x7ff38000u)|(probe_source<<20)|(probe_clock<<15);
#endif
    REG_WRITE(0x60033d5c,ctrl);
    REG_WRITE(0x60033d5c,ctrl|(1u<<19));
    REG_WRITE(0x60033d5c,ctrl);
    while(!(REG_READ(0x60033d5c)&(1u<<18)) && esp_timer_get_time()-start<20000){}
    bool done=(REG_READ(0x60033d5c)&(1u<<18))!=0;
    uint32_t elapsed=esp_timer_get_time()-start;
    REG_WRITE(0x60033d5c,0);
    REG_WRITE(SRAM_OWNER_REG,owner);
    rx_filter_restore();
#ifdef SAMPLE_RATE_PROBE
    if(probe_adc<4)rom_chip_i2c_writeReg(0x66,0,4,adc_saved);
#endif
    *elapsed_us=elapsed;
    if(!done)return false;
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au)return false;
    }
    return true;
}

static bool capture(unsigned n,unsigned divider,unsigned format) {
    if(divider!=0 && divider!=1 && divider!=6){reply("ERR rate\n");return false;}
    uint32_t elapsed;
    if(!capture_raw(n,divider,&elapsed)){reply("ERR capture_timeout\n");return false;}
    size_t bytes=wire_size(n,format);
    if(format==16)pack_iq8(n);else if(format==20)pack_iq(n);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    char h[96];
    snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIu32 "\n",n,crc,elapsed);
    return send_bytes(h,strlen(h)) && send_bytes(IQ_BUFFER,bytes);
}

static void handle_command(char *line) {
    if(!strcmp(line,"TRANSPORT?")) {
        char answer[64];
        snprintf(answer,sizeof(answer),"TRANSPORT %s %u\n",
                 burst_serial_port()==BURST_SERIAL_UART?"UART":"USB",burst_serial_baud());
        reply(answer);return;
    }
#ifdef FILTER_REGISTER_PROBE
        if(filter_probe_command(line))return;
#endif
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
             * is a separate capture, with RF gaps during USB transfer. */
            bool ok=true;
            for(unsigned j=0;j<repeats && ok;j++){ok=capture(n,rate,crc);vTaskDelay(1);}
            if(ok)reply("END\n");
        }
#ifdef SAMPLE_RATE_PROBE
        else if(sscanf(line,"RXPROBE %u %u %c",&n,&rate,&extra)==2 && n<2048 && rate<8) {
            probe_source=n;probe_clock=rate;probe_capture=true;capture(16380,0,20);probe_capture=false;
        }
        else if(sscanf(line,"ADCCLOCK %u %c",&n,&extra)==1 && n<=4) {probe_adc=n;reply("OK\n");}
        else if(!strcmp(line,"ADCCLOCK?")){char h[64];snprintf(h,sizeof(h),"ADC %u\n",rom_chip_i2c_readReg(0x66,0,4));reply(h);}
#endif
        else if(!strcmp(line,"CAPS")) {
            reply("CAPS RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
                  "DUALSERIAL "
#endif
                  "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8\n");
        }
        else if(sscanf(line,"BANDWIDTH %u %c",&n,&extra)==1 && (!n || (n>=RX_BANDWIDTH_MIN && n<=RX_BANDWIDTH_MAX))) {
            rx_filter=rx_bandwidth_dcap(n);reply("OK\n");
        }
        else if(!strcmp(line,"LPF AUTO")){rx_filter=-1;reply("OK\n");}
        else if(sscanf(line,"LPF %u %c",&n,&extra)==1 && n<=63){rx_filter=n;reply("OK\n");}
        else if(!strcmp(line,"LPF?")){
            char answer[80];snprintf(answer,sizeof(answer),"LPF %d %u %u\n",rx_filter,
                rom_chip_i2c_readReg(0x67,0,4)&63,rom_chip_i2c_readReg(0x67,0,5)&63);reply(answer);
        }
#ifdef S3_RF_PROBE
        else if(!strcmp(line,"WORDS?")){char h[120];snprintf(h,sizeof(h),"WORDS %08x %08x %08x %08x OWNER %08x\n",(unsigned)IQ_BUFFER[0],(unsigned)IQ_BUFFER[1],(unsigned)IQ_BUFFER[2],(unsigned)IQ_BUFFER[3],(unsigned)REG_READ(SRAM_OWNER_REG));reply(h);}
        else if(!strcmp(line,"MEMTEST")){for(unsigned j=0;j<IQ_WORDS;j++)IQ_BUFFER[j]=0x12345678u+j;reply("OK\n");}
        else if(sscanf(line,"RXWIDE %u %c",&n,&extra)==1 && n<2){rx_wide=n;reply("OK\n");}
        else if(sscanf(line,"RXPREP %u %c",&n,&extra)==1 && n<4){rx_prep=n;rx_ready=false;reply("OK\n");}
        else if(sscanf(line,"RXPACK %u %u %c",&n,&rate,&extra)==2 && n<=60 && rate<=1){rx_pack=n;rx_agc=rate;reply("OK\n");}
        else if(sscanf(line,"RXSRC %u %c",&n,&extra)==1 && n<256){rx_source=n;reply("OK\n");}
        else if(sscanf(line,"RXMODE %u %c",&n,&extra)==1 && n<8){rx_mode=n;reply("OK\n");}
        else if(sscanf(line,"RXFLAG %u %c",&n,&extra)==1 && n<2){rx_flag=n;reply("OK\n");}
        else if(sscanf(line,"RXCLOCK %u %c",&n,&extra)==1 && n<2){rx_clock=n;reply("OK\n");}
        else if(!strcmp(line,"RXREGS?")){
            for(unsigned a=0x60033d50;a<=0x60033d98;a+=4){char h[60];snprintf(h,sizeof(h),"REG %08x %08x\n",a,(unsigned)REG_READ(a));reply(h);}reply("END\n");
        }
        else if(!strcmp(line,"RFREG?")){char h[120];snprintf(h,sizeof(h),"RFREG %08x %08x %08x\n",(unsigned)REG_READ(0x60033d5c),(unsigned)REG_READ(0x60033d60),(unsigned)REG_READ(0x60033d64));reply(h);}
#endif
#ifdef S3_RF_PROBE
        else if(sscanf(line,"FREQEX %u %c",&n,&extra)==1 && n>=100 && n<=6000) {
            frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
        }
#endif
        else if(!strcmp(line,"RANGE?")) {
            char answer[64];snprintf(answer,sizeof(answer),"RANGE %u %u 1\n",S3_FREQ_MIN,S3_FREQ_MAX);reply(answer);
        }
        else if(!strcmp(line,"INFO")) reply("S3SDR 6 burst 16380\n");
        else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 && ((n>=S3_FREQ_MIN && n<=S3_FREQ_MAX))) {
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
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=8192,.rx_buffer_size=8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*",ESP_LOG_NONE);
    /* USB may be unplugged when the host uses the UART bridge. */
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    burst_serial_init();
#if CONFIG_ESP_SDR_LCD_ES3C28P
    lcd_waterfall_init();
#endif
    char line[128];
    int owner=-1;
    int64_t lease_deadline=0;
    for(;;) {
        if(esp_timer_get_time()>=lease_deadline)owner=-1;
        int status=burst_serial_poll_line(line,sizeof(line));
        if(!status){
#if CONFIG_ESP_SDR_LCD_ES3C28P
            /* Standalone display while no serial host holds the radio. */
            uint32_t elapsed;
            if(owner<0 && capture_raw(IQ_WORDS,0,&elapsed))
                lcd_waterfall_frame(IQ_BUFFER,IQ_WORDS,frequency_mhz,80);
#endif
            vTaskDelay(1);continue;
        }
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
