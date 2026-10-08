/* ESP32-S3 burst SDR over native USB Serial/JTAG and UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
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
#include "burst_gpio.h"
#include "burst_version.h"
#include "rx_recalibration.h"
#include "rx_tuning.h"
#include "rx_lo.h"
#include "esp_rom_sys.h"
#include "ring_capture.h"

/* Vendor S3 adctrig uses the 64 KiB aperture at 0x3fcd0000 (MAC_DUMP_USAGE=4).
 * The continuous ring also uses the two banks below it. Keep all three, in
 * both DRAM and IRAM aliases, out of the heap and static sections. */
SOC_RESERVE_MEMORY_REGION(RING_BANK_BASE, RING_BANK_END, s3_rf_dump);
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
static void s3_tune(unsigned mhz);
static int s3_fofs; /* FOFS: PLL offset in kHz, applied from the next tune */
static void s3_tune(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan=rx_lo_plan(mhz);
    /* A PLL offset also requires direct tuning on Wi-Fi channel frequencies. */
    bool channel=!s3_fofs && ((mhz>=2412 && mhz<=2472 && (mhz-2412)%5==0)||mhz==2484);
    rx_lo_select(false);
    set_chanfreq(channel?mhz:2412,0);
    if(!channel)set_rf_freq_offset(0,plan.mhz,plan.offset_khz+s3_fofs);
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
    if(rx_ready && !rx_recalibration_stale())return;
    if(rx_prep==1){esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);force_rx_gain(1,55,0);rx_ready=true;return;}
    if(rx_prep==2){esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);rx_ready=true;return;}
    s3_tune(frequency_mhz);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    rx_ready=true;
}
/* A forced index update alone can leave continuous capture using stale RX
 * state until the next tune. Apply gain changes through the same receiver
 * setup as FREQ, before acknowledging the command. */
static void gain_reconfigure(void) {
    rx_ready=false;
    prepare_rx();
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
static bool capture(unsigned n,unsigned divider,unsigned format) {
    if(divider!=0 && divider!=1 && divider!=6){reply("ERR rate\n");return false;}
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
    if(!done){reply("ERR capture_timeout\n");return false;}
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au){reply("ERR capture_timeout\n");return false;}
    }
    size_t bytes=wire_size(n,format);
    if(format==16)pack_iq8(n);else if(format==20)pack_iq(n);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    char h[96];
    snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIu32 "\n",n,crc,elapsed);
    return send_bytes(h,strlen(h)) && send_bytes(IQ_BUFFER,bytes);
}

/* Continuous modes (ring_capture.c). Reports end with one text line:
 * <TAG> status detail units pairs elapsed_us late_max work_max_cycles
 *       frames drops abandoned ffts stopped_by_host */
static void ring_report(const char *tag,const ring_result_t *r) {
    char h[224];
    snprintf(h,sizeof(h),"%s %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu64 " %" PRIu32
             " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %u\n",tag,r->status,r->detail,
             r->units,r->pairs,r->elapsed_us,r->late_max,r->work_max,r->frames,r->drops,r->abandoned,
             r->ffts,(unsigned)r->stopped_by_host);
    reply(h);
}
/* RINGCAP payload: "RINGDATA units rate_hz n0 n1 n2 crc32\n", then the units'
 * raw 32-bit IQ words back to back (gapless), then the RINGCAP report. */
static void ring_send_capture(const ring_result_t *r,unsigned rate) {
    uint32_t crc=0;
    for(unsigned u=0;u<r->units;u++) {
        const uint32_t *p=ring_capture_bank(r->cap[u].bank);
        unsigned first=r->cap[u].first,n=r->cap[u].count,head=RING_PAIRS-first;
        if(head>n)head=n;
        crc=esp_rom_crc32_le(crc,(const uint8_t *)(p+first),head*4);
        crc=esp_rom_crc32_le(crc,(const uint8_t *)p,(n-head)*4);
    }
    char h[128];
    snprintf(h,sizeof(h),"RINGDATA %" PRIu32 " %u %" PRIu32 " %" PRIu32 " %" PRIu32 " %08" PRIx32 "\n",
             r->units,ring_capture_rate_hz(rate),r->cap[0].count,r->units>1?r->cap[1].count:0,
             r->units>2?r->cap[2].count:0,crc);
    reply(h);
    for(unsigned u=0;u<r->units;u++) {
        const uint32_t *p=ring_capture_bank(r->cap[u].bank);
        unsigned first=r->cap[u].first,n=r->cap[u].count,head=RING_PAIRS-first;
        if(head>n)head=n;
        (void)send_bytes(p+first,head*4);
        if(n>head)(void)send_bytes(p,(n-head)*4);
    }
}
static bool ring_command(const char *line) {
    unsigned ms,rate,stride,upf,mode,units,nfft,sflag=0,n;char extra;int k;
    if(!strcmp(line,"DUAL?")){char h[48];snprintf(h,sizeof(h),"DUAL %u %u\n",(unsigned)ring_capture_dual_active()*(ring_capture_assist?1u:2u),(unsigned)ring_capture_core1_alive());reply(h);return true;}
    {int off;if(sscanf(line,"FOFS %d %c",&off,&extra)==1){s3_fofs=off;rx_ready=false;prepare_rx();reply("OK\n");return true;}}
    if(!strcmp(line,"DC?")){char h[16];snprintf(h,sizeof(h),"DC %u\n",ring_capture_dc_mode);reply(h);return true;}
    if(sscanf(line,"DC %u %c",&n,&extra)==1 && n<2){ring_capture_dc_mode=n;reply("OK\n");return true;}
    if(!strcmp(line,"ASSIST?")){char h[48];snprintf(h,sizeof(h),"ASSIST %u %" PRIu32 "\n",(unsigned)ring_capture_assist,ring_capture_c0_blocks);reply(h);return true;}
    if(sscanf(line,"DUAL %u %c",&n,&extra)==1 && n<3){ring_capture_set_dual(n!=0);ring_capture_assist=n==1;reply("OK\n");return true;}
    ring_config_t c={0};
    const char *tag;
    bool usb=burst_serial_port()==BURST_SERIAL_USB;
    if(!strcmp(line,"SPECINFO?")) {
        /* Worker profiles include 512 bins; retain the staged fallback when
         * the worker is unavailable or explicitly disabled. */
        if(ring_capture_dual_active())
            reply("SPECINFO {\"continuous\":true,\"transports\":[\"USB\"],\"profiles\":["
                  "[16000000,6,256,2,1],[16000000,6,512,2,2],[16000000,6,1024,2,4],[16000000,6,2048,3,7],"
                  "[40000000,1,256,5,3],[40000000,1,512,5,5],[40000000,1,1024,5,9],[40000000,1,2048,7,17],"
                  "[80000000,0,256,10,6],[80000000,0,512,12,10],[80000000,0,1024,14,16],[80000000,0,2048,14,30]]}\n");
        else
        reply("SPECINFO {\"continuous\":true,\"transports\":[\"USB\"],\"profiles\":["
              "[16000000,6,256,4,2],[16000000,6,512,6,3],[16000000,6,1024,8,4],[16000000,6,2048,6,8],"
              "[40000000,1,256,12,4],[40000000,1,512,12,6],[40000000,1,1024,18,10],[40000000,1,2048,10,20],"
              "[80000000,0,256,48,8],[80000000,0,512,32,12],[80000000,0,1024,24,16],[80000000,0,2048,16,24]]}\n");
        return true;
    }
    if(!strcmp(line,"RINGINFO?")) {
        /* The ring reserves 192 KiB of SRAM; report what the heap kept. */
        char h[96];
        snprintf(h,sizeof(h),"RINGINFO %u %u %u %u\n",(unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),RING_BANKS,RING_THRESHOLD);
        reply(h);return true;
    }
    if(sscanf(line,"RING %u %u %c",&ms,&rate,&extra)==2) {
        c.mode=RING_MODE_STATS;c.rate=rate;c.duration_ms=ms;tag="RING";
        if(rate!=0 && rate!=1 && rate!=6){reply("ERR rate\n");return true;}
        if((!ms && !usb) || ms>86400000u){reply("ERR args\n");return true;}
    } else if(sscanf(line,"RINGCAP %u %u %c",&units,&rate,&extra)==2) {
        c.mode=RING_MODE_CAPTURE;c.rate=rate;c.capture_units=units;tag="RINGCAP";
        if(rate!=0 && rate!=1 && rate!=6){reply("ERR rate\n");return true;}
        if(!units || units>RING_BANKS){reply("ERR args\n");return true;}
    } else if(!strncmp(line,"SPEC ",5) &&
              (k=sscanf(line,"SPEC %u %u %u %u %u %u %u %c",&ms,&stride,&upf,&mode,&rate,&nfft,&sflag,&extra))>=4 && k<=7) {
        /* Optional 5th field: rate code 6 = 16, 1 = 40, 0 = 80 Msps (default 16). */
        if(k==4)rate=6;
        if(k<6)nfft=256;
        if(!ring_capture_valid_nfft(nfft)){reply("ERR nfft\n");return true;}
        if(rate!=0 && rate!=1 && rate!=6){reply("ERR rate\n");return true;}
        c.mode=RING_MODE_SPEC;c.rate=rate;c.nfft=nfft;c.duration_ms=ms;c.stride=stride;c.units_per_frame=upf;
        c.max_hold=mode==1;c.stats=k>=7&&sflag;tag="SPECEND";
        if(!usb){reply("ERR transport\n");return true;}
        if(!stride || stride>64 || !upf || upf>1000 || mode>1 || sflag>1 || ms>86400000u){reply("ERR args\n");return true;}
        char h[80];
        snprintf(h,sizeof(h),"SPEC %u %u %u %u\n",nfft,ring_capture_rate_hz(rate),RING_THRESHOLD,frequency_mhz);
        reply(h);
    } else if(sscanf(line,"IQS %u %u %u %u %u %u %c",&ms,&stride,&nfft,&rate,&upf,&mode,&extra)>=4) {
        /* IQS ms dec bits rate [shift] [mode]: continuous decimated IQ (IQS1 frames).
         * mode 0: FIR at 0 Hz IF; mode 2: +fs/4 shift first (LO tuned fs/4 below).
         * Default shift scales the 15-bit FIR output to the requested bits. */
        int kk=sscanf(line,"IQS %u %u %u %u %u %u",&ms,&stride,&nfft,&rate,&upf,&mode);
        if(kk<5)upf=nfft<15u?15u-nfft:0u;
        if(kk<6)mode=0;
        if(mode!=0 && mode!=2){reply("ERR mode\n");return true;}
        if(rate!=0 && rate!=1 && rate!=6){reply("ERR rate\n");return true;}
        if(!usb){reply("ERR transport\n");return true;}
        c.mode=RING_MODE_IQ;c.rate=rate;c.duration_ms=ms;c.iq_dec=stride;c.iq_bits=nfft;c.iq_shift=upf;c.iq_rot=mode==2;tag="IQSEND";
        char h[96];
        snprintf(h,sizeof(h),"IQS %u %u %u %u %u %u\n",ring_capture_rate_hz(rate),stride,nfft,upf,mode,frequency_mhz);
        reply(h);
    } else return false;
    ring_result_t r;
    prepare_rx();
    rx_filter_apply();
    ring_capture_run(&c,&r);
    rx_filter_restore();
    if(c.mode==RING_MODE_CAPTURE && !r.status)ring_send_capture(&r,rate);
    ring_report(tag,&r);
    return true;
}

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
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
        if(ring_command(line))return;
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
            reply("CAPS VERSION GPIO UARTBAUD RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
                  "DUALSERIAL "
#endif
                  "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8 RING SPEC SPECN SPECCAPS SPECSTAT DCT\n");
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
    ring_capture_init();
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
