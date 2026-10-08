/* Shared ESP32-C5/C6/C61 burst receiver. Chip differences: target-selected chip.h. */
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
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "riscv/rv_utils.h"
#include "hal/usb_serial_jtag_ll.h"

#ifndef CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_IDF_TARGET_ESP32C6 0
#endif
#include "chip.h"
#include "burst_serial.h"
#include "burst_gpio.h"
#include "burst_version.h"
#include "spectrum.h"
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#include "ring_capture.h"
#endif
#include "rx_tuning.h"
#include "rx_recalibration.h"
extern void phy_stop_tx_tone(unsigned);
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(unsigned);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(unsigned);
extern void phy_chip_set_chan(unsigned,unsigned);
extern void phy_rx_filter_mode(unsigned);
static unsigned frequency_mhz=2412;
static bool rx_ready;
#if CONFIG_IDF_TARGET_ESP32C5
static unsigned rx_channel_mode;
#endif
#if !CONFIG_IDF_TARGET_ESP32C6
static int rx_filter=-1; /* -1 restores the PHY-calibrated automatic mode. */
#endif
static int rx_analog_filter=-1;
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#define RX_FILTER_REG 4u
#else
#define RX_FILTER_REG 6u
#endif
extern unsigned phy_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void phy_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
/* RX-only analog capacitance. Preserve PHY calibration between snapshots,
 * across retunes; zero is the widest tested code, not bypass. */
static void rx_analog_apply(unsigned saved[2]) {
    for(unsigned j=0;j<2;j++) {
        saved[j]=phy_chip_i2c_readReg(0x67,1,RX_FILTER_REG+j);
        if(rx_analog_filter>=0)phy_i2c_writeReg(0x67,1,RX_FILTER_REG+j,(saved[j]&~63u)|(unsigned)rx_analog_filter);
    }
}
static void rx_analog_restore(const unsigned saved[2]) {
    if(rx_analog_filter>=0)for(unsigned j=0;j<2;j++)phy_i2c_writeReg(0x67,1,RX_FILTER_REG+j,saved[j]);
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
    static unsigned calibrated_mhz;
    if(rx_ready && !rx_recalibration_stale())return;
#if CONFIG_IDF_TARGET_ESP32C61
    burst_gain_mirror(-1);
    if(gain_defaults_saved) {
        REG_WRITE(0x600a7094u,gain_init_saved);
        REG_WRITE(0x600a713cu,gain_threshold_saved);
        gain_defaults_saved=false;
    }
#endif
    if (calibrated_mhz != frequency_mhz || rx_recalibration_stale()) {
        rx_recalibrate(frequency_mhz);
        calibrated_mhz = frequency_mhz;
    }
#if CONFIG_IDF_TARGET_ESP32C5
    phy_chip_set_chan(frequency_mhz,rx_channel_mode);
#else
    phy_chip_set_chan(frequency_mhz,0);
#endif
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
#if !CONFIG_IDF_TARGET_ESP32C6
    if(rx_filter>=0)phy_rx_filter_mode((unsigned)rx_filter);
#endif
    gain_apply();
#if CONFIG_IDF_TARGET_ESP32C6
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
#endif
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
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
/* Volatile, bounded dump-clock/source investigation; excluded from releases. */
static unsigned probe_source,probe_clock,probe_adc=4;
extern void phy_adc_rate_set(unsigned);
extern unsigned phy_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void phy_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
static bool probe_capture;
#endif
static bool acquire_iq(unsigned n,unsigned divider,unsigned *capture_us) {
    prepare_rx();
    
    for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
    for(unsigned j=0;j<4;j++)IQ_BUFFER[n+j]=0x5a5aa5a5u^j;
    unsigned analog_saved[2];rx_analog_apply(analog_saved);
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    unsigned adc_saved=phy_chip_i2c_readReg(0x66,0,4);
    uint32_t adc_digital_saved=REG_READ(0x600a0448);
    if(probe_adc<2)phy_adc_rate_set(probe_adc);
#endif
    bool done=true;
    int64_t start=esp_timer_get_time();
    /* Vendor selector 0 maps to raw source 15 and pulses the software trigger.
     * In particular, do NOT set CTRL bit 17 as in the C61 continuous backend:
     * on this C5 it produced only a short, incomplete snapshot. */
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    if(probe_capture && probe_source>0) {
        /* Seed stock packing/clock setup, then bounded raw-source capture. */
        adctrig(255,0,0,probe_clock*2u,0,0,0,0,0);
        for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
        uint32_t mode=REG_READ(0x600a9008);
        REG_WRITE(0x600a9008,(mode&~0x001e0000u)|((probe_source-1)<<17));
        REG_WRITE(SRAM_OWNER_REG,(owner&~0xf00u)|0x10200u);
        uint32_t ctrl=0x80000000u|n;
        REG_WRITE(0x600a9004,ctrl);
        REG_WRITE(0x600a9004,ctrl|(1u<<19));REG_WRITE(0x600a9004,ctrl);
        int64_t wait_start=esp_timer_get_time();
        while(!(REG_READ(0x600a9004)&(1u<<22)) && esp_timer_get_time()-wait_start<20000){}
        REG_WRITE(0x600a9004,0);REG_WRITE(0x600a9008,mode);
    } else done=stock_capture(n,probe_capture?probe_clock:divider);
#else
    done=stock_capture(n,divider);
#endif
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-start);
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    if(probe_adc<2){phy_i2c_writeReg(0x66,0,4,adc_saved);REG_WRITE(0x600a0448,adc_digital_saved);}
#endif
    REG_WRITE(SRAM_OWNER_REG,owner);
    rx_analog_restore(analog_saved);
    if(!done){reply("ERR capture_timeout\n");return false;}
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au){reply("ERR capture_timeout\n");return false;}
    }
    for(unsigned j=0;j<4;j++) {
        if(IQ_BUFFER[n+j]!=(0x5a5aa5a5u^j)){reply("ERR capture_overrun\n");return false;}
    }
    *capture_us = elapsed;
    return true;
}

static bool spectrum_acquire(unsigned n, unsigned rate, const uint32_t **data, unsigned *elapsed) {
    bool ok = acquire_iq(n, rate, elapsed);
    *data = IQ_BUFFER;
    return ok;
}

#include "ring_probe.h"

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

#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
static bool ring_test(const char *line) {
    unsigned ms,rate,stride=1,upf=1,det=0,n=256,stats=0; char extra;
    int fields=sscanf(line,"SPEC %u %u %u %u %u %u %u %c",&ms,&stride,&upf,&det,&rate,&n,&stats,&extra);
    bool spec=fields==6 || fields==7;
    if(spec && (n!=256 || burst_serial_port()==BURST_SERIAL_UART))return false;
#ifdef RING_PROBE
    if(!spec)spec=sscanf(line,"RINGSPEC %u %u %u %u %u %u %c",&ms,&stride,&upf,&det,&rate,&n,&extra)==6;
    if(!spec && sscanf(line,"RINGTEST %u %u %c",&ms,&rate,&extra)!=2)return false;
#else
    if(!spec)return false;
#endif
    if(ms>86400000u || rate>5 || !stride || !upf || det>1 || stats>1 || stride>64 || upf>1000 || n!=256
#if CONFIG_IDF_TARGET_ESP32C6
        || rate!=0
#endif
    ){reply("ERR spec_args\n");return true;}
    if(spec){ring_capture_init();char h[80];snprintf(h,sizeof(h),"SPEC %u %u %u %u\n",n,ring_capture_rate_hz(rate),RING_THRESHOLD,frequency_mhz);reply(h);}
    prepare_rx();
    unsigned saved[2];rx_analog_apply(saved);
    ring_config_t cfg={.mode=spec?RING_MODE_SPEC:RING_MODE_STATS,.rate=rate,.duration_ms=ms,.nfft=n,.stride=stride,.units_per_frame=upf,.max_hold=det==1,.stats=stats!=0};
    ring_result_t r;ring_capture_run(&cfg,&r);rx_analog_restore(saved);
    char h[200];snprintf(h,sizeof(h),"%s %u %u %u %llu %llu %u %u %u %u %u %u %u\n",spec?"SPECEND":"RINGTEST",(unsigned)r.status,(unsigned)r.detail,
        (unsigned)r.units,(unsigned long long)r.pairs,(unsigned long long)r.elapsed_us,(unsigned)r.late_max,(unsigned)r.work_max,(unsigned)r.frames,(unsigned)r.drops,(unsigned)r.abandoned,(unsigned)r.ffts,r.stopped_by_host);
    reply(h);return true;
}
#endif

static void handle_command(char *line);

void app_main(void) {

    esp_log_level_set("*",ESP_LOG_NONE);
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=512,.rx_buffer_size=512};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    cfg.static_rx_buf_num=4;
#endif
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*",ESP_LOG_NONE);
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    burst_serial_init();
    char line[128];int owner=-1;int64_t lease_deadline=0;
    for(;;) {
        if(esp_timer_get_time()>=lease_deadline)owner=-1;
        int status=burst_serial_poll_line(line,sizeof(line));
        if(!status){vTaskDelay(1);continue;}
        int port=burst_serial_port();
        if(owner>=0 && owner!=port){reply("ERR busy\n");continue;}
        if(status<0){reply("ERR command_length\n");continue;}
        owner=port;
        handle_command(line);
        if(!strcmp(line,"RELEASE"))owner=-1;
        lease_deadline=esp_timer_get_time()+5000000;
    }
}

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef RING_PROBE
    if(ring_probe_command(line)) return;
#endif
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    if(ring_test(line))return;
#endif
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
    if(!strcmp(line,"RELEASE")){reply("OK\n");return;}
    if(!strcmp(line,"TRANSPORT?")) {
        char h[64];snprintf(h,sizeof(h),"TRANSPORT %s %u\n",
            burst_serial_port()==BURST_SERIAL_UART?"UART":"USB",burst_serial_baud());
        reply(h);return;
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
#if defined(C5_TUNE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
        else if(sscanf(line,"FREQEX %u %c",&n,&extra)==1 && n>=100 && n<=7500) {
            frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
        }
#endif
        else if(sscanf(line,"RXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
                n>=256 && n<=IQ_WORDS && rate<=(CONFIG_IDF_TARGET_ESP32C6?0:5) && repeats>0 && repeats<=1000 && (crc==16 || crc==20)) {
            /* Format 16 = IQ8, 20 = IQ10 packed. Each frame
             * is a separate capture, with RF gaps during USB transfer. */
            bool ok=true;
            for(unsigned j=0;j<repeats && ok;j++){ok=capture(n,rate,crc);vTaskDelay(1);}
            if(ok)reply("END\n");
        }
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
        else if(sscanf(line,"RXPROBE %u %u %c",&n,&rate,&extra)==2 && n<17 && rate<8) {
            probe_source=n;probe_clock=rate;probe_capture=true;capture(16380,0,20);probe_capture=false;
        }
        else if(sscanf(line,"ADCCLOCK %u %c",&n,&extra)==1 && (n<2 || n==4)) {probe_adc=n;reply("OK\n");}
#endif
        else if(!strcmp(line,"CAPS")) {
            reply("CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS GAIN HWAGC IQ8 SERIALLEASE"
                  " TUNEEXT"
#if !CONFIG_IDF_TARGET_ESP32C6
                  " LPF LPF12"
#endif
                  " ALPF"
#if CONFIG_ESP_SDR_UART_ENABLED
                  " DUALSERIAL"
#endif
                  "\n");
        }
        else if(!strcmp(line,"RANGE?")){reply(RX_TUNING_RANGE_REPLY);}
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
        else if(sscanf(line,"BANDWIDTH %u %c",&n,&extra)==1 && (!n || (n>=RX_BANDWIDTH_MIN && n<=RX_BANDWIDTH_MAX))) {
            rx_analog_filter=rx_bandwidth_dcap(n);
#if CONFIG_IDF_TARGET_ESP32C5
            unsigned mode=rx_bandwidth_phy_mode(n);
            if(mode!=rx_channel_mode) {
                rx_channel_mode=mode;rx_ready=false;prepare_rx();
            }
#endif
            reply("OK\n");
        }
#endif
        else if(!strcmp(line,"ALPF AUTO")){rx_analog_filter=-1;reply("OK\n");}
        else if(sscanf(line,"ALPF %u %c",&n,&extra)==1 && n<=63){rx_analog_filter=(int)n;reply("OK\n");}
        else if(!strcmp(line,"ALPF?")) {
            prepare_rx();char answer[64];snprintf(answer,sizeof(answer),"ALPF %d %u %u\n",rx_analog_filter,
                phy_chip_i2c_readReg(0x67,1,RX_FILTER_REG),phy_chip_i2c_readReg(0x67,1,RX_FILTER_REG+1));reply(answer);
        }
#if !CONFIG_IDF_TARGET_ESP32C6
        else if(!strcmp(line,"LPF AUTO")){rx_filter=-1;rx_ready=false;prepare_rx();reply("OK\n");}
        else if(sscanf(line,"LPF %u %c",&n,&extra)==1 && (n==0 || n==4 || n==8 || n==12)) {
            rx_filter=(int)n;rx_ready=false;prepare_rx();reply("OK\n");
        }
        else if(!strcmp(line,"LPF?")) {
            char answer[64];snprintf(answer,sizeof(answer),"LPF %d %u\n",rx_filter,(unsigned)((REG_READ(0x600a0430)>>18)&15));reply(answer);
        }
#endif
        else if(!strcmp(line,"INFO")) {
            char h[64];snprintf(h,sizeof(h),BURST_ID " 6 burst %u\n",IQ_WORDS);reply(h);
        }
        else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 && frequency_valid(n)) {
            frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
        } else if(((!strncmp(line,"CAP ",4) && sscanf(line,"CAP %u %u %c",&n,&rate,&extra)==2) ||
                   (!strncmp(line,"CAP20 ",6) && sscanf(line,"CAP20 %u %u %c",&n,&rate,&extra)==2)) &&
                   n>=256 && n<=IQ_WORDS && rate<=(CONFIG_IDF_TARGET_ESP32C6?0:5)) capture(n,rate,!strncmp(line,"CAP20 ",6)? (iq8?16:20):0);
        else reply("ERR command\n");
}
