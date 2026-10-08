/* ESP32-S31 RX-only backend for the ESP-SDR burst protocol.
 * Bounded snapshots and continuous dual-core SIMD spectra over native USB.
 */
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "burst_serial.h"
#include "burst_gpio.h"
#include "burst_version.h"
#include "spectrum.h"
#include "rx_tuning.h"
#include "rx_bandwidth.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/hp_system_reg.h"
#include "esp_ipc_isr.h"
#include "esp_cpu.h"
#include "esp_system.h"
#include "esp_phy_cert_test.h"
#include "driver/usb_serial_jtag.h"
#include "heap_memory_layout.h"

/* RF ownership covers complete 128 KiB groups. Continuous spectra alternate
 * both groups; snapshots use the upper group. Exclude them from the heap and
 * enforce the same boundary for static sections in sram_guard.ld. */
SOC_RESERVE_MEMORY_REGION(0x2f040000, 0x2f07f170, s31_rf_dump);
#define DUMP_CTRL 0x20109004u
#define DUMP_MODE 0x20109008u
#define DUMP_MAC 0x2010900cu
#define DUMP_BYTES 0x20109018u
#define RX_GAIN 0x2010702cu
#define CAPACITY 26624u
#define MAX_SAMPLES 16380u
#define SETTLE_SAMPLES 256u
#define SENTINEL 0xa55aa55au
_Static_assert((MAX_SAMPLES + SETTLE_SAMPLES) < CAPACITY, "S31 dump stop margin");
_Static_assert(MAX_SAMPLES * 4 <= 0x10000, "S31 packing buffer fits lower guard");
_Static_assert(0x2f060000u + (CAPACITY+4)*4 <= 0x2f07afb0u, "S31 dump excludes ROM memory");
static uint32_t *const samples = (void *)0x2f050000;
static volatile uint32_t *const dump = (void *)0x2f060000;
static portMUX_TYPE dump_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned frequency_mhz = 2412;
static unsigned reset_ctrl;
int cmd_parse(char *cmd,char *name,int *argc,char **argv) {
    (void)cmd;(void)name;(void)argc;(void)argv;return -1;
}
static unsigned gain_max, gain_code = 40;
static bool hardware_agc = true;
/* S31 BBTOP RX filter: registers 4/5, six-bit capacitor DAC. */
static int rx_filter = -1;
extern unsigned phy_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
static unsigned filter_saved[2];
static void filter_apply(void) {
    for (unsigned j=0; j<2; j++) {
        filter_saved[j]=phy_i2c_readReg(0x67, 1, 4+j);
        if (rx_filter>=0) phy_i2c_writeReg(0x67, 1, 4+j, (filter_saved[j]&~63u)|(unsigned)rx_filter);
    }
    esp_rom_delay_us(10);
}
static void filter_restore(void) {
    if (rx_filter>=0) for (unsigned j=0; j<2; j++)
        phy_i2c_writeReg(0x67, 1, 4+j, filter_saved[j]);
}
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(int);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(int);

static void reply(const char *fmt, ...) {
    char text[256];
    va_list ap;
    va_start(ap, fmt);
    int length = vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    if (length > 0) burst_serial_send(text, length < sizeof(text) ? length : sizeof(text)-1);
}

extern void phy_chip_set_chan(unsigned, unsigned);
extern void phy_set_freq(unsigned, int);
#include "tuning.h"
extern void phy_loopback_mode_en(unsigned);
extern void phy_bb_bss_cbw40_dig(unsigned);
extern void phy_bb_cbw_chan_cfg(unsigned);
extern void phy_wifi_fbw_sel(unsigned);
extern void phy_force_rx_gain(unsigned, unsigned);
extern void phy_rfrx_sat_rst(unsigned);
extern void phy_pbus_debugmode(void);
extern void phy_set_txclk_en(unsigned);
extern void phy_set_rx_gain_table(unsigned, unsigned);
extern unsigned char phy_param[];
extern void burst_gain_mirror(int);
static unsigned gain_init, gain_threshold;
static void prepare_rx(void) {
    if (rx_recalibration_stale()) s31_tune(frequency_mhz);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    /* PBUS work-mode setup releases forced RX gain. Restore the requested
     * mode after it, including when starting an IQ capture or spectrum run. */
    phy_rfrx_sat_rst(hardware_agc);
    phy_force_rx_gain(!hardware_agc, hardware_agc ? 0 : gain_code);
}

/* No PHY/I2C, allocation, logging or scheduler calls inside the open gate.
 * The writer uses continuous aperture mode even for snapshots: the vendor's
 * finite test mode writes at TCM offset zero, overlapping the application.
 * Stop before the first wrap, retaining 256 initial samples for settling.
 * The remaining >9K words give ample margin for MMIO stop latency. */
static void IRAM_ATTR dump_init(void) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&dump_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_CLR_BIT(0x201008cc, 0x00780000);
    REG_WRITE(0x201070b8, (REG_READ(0x201070b8)&~7u)|1u);
    REG_SET_BIT(0x20100800, BIT(2));
    REG_WRITE(0x20109c04, UINT32_MAX);
    REG_CLR_BIT(DUMP_MAC, BIT(31));
    REG_SET_BIT(HP_SYSTEM_TCM_RAM_PWR_CTRL0_REG, HP_SYSTEM_REG_HP_SYSTEM_TCM_CLK_FORCE_ON);
    REG_SET_BIT(0x20109c04, BIT(31)|BIT(21));
    REG_SET_BIT(0x20109c08, BIT(31));
    REG_SET_BIT(0x20109c14, 0xe400);
    REG_WRITE(0x20109c0c, (REG_READ(0x20109c0c)&~0xff00f000u)|0x44004000u);
    REG_SET_BIT(0x20109c10, BIT(31));
    REG_CLR_BIT(0x20109c10, BIT(31));
    /* Preserve implementation-specific reset fields (observed 0x03c00000).
     * Clearing them stalls the writer, even with all dump clocks enabled. */
    reset_ctrl=REG_READ(DUMP_CTRL);
    REG_SET_BIT(0x20100800, 4);
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE)&~0x01fe0000u)|(3 << 17));
    REG_WRITE(DUMP_BYTES, (REG_READ(DUMP_BYTES)&~0xffffffu) | 24 | (25<<6) | (26<<12) | (27<<18) | BIT(24));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu) | CAPACITY | BIT(17) | BIT(18));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu) | CAPACITY | BIT(17));
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);
    esp_ipc_isr_release_other_cpu();
}

static unsigned IRAM_ATTR acquire(unsigned n, unsigned divider, unsigned *end) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&dump_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE)&~0x01fe0000u)|(divider<<21)|(3u<<17));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17)|BIT(18));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17));
    unsigned start = esp_cpu_get_cycle_count();
    REG_SET_BIT(DUMP_CTRL, BIT(31));
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));
    unsigned current=0;
    while (current<n+SETTLE_SAMPLES && esp_cpu_get_cycle_count()-start < 3000000u) {
        current=REG_READ(DUMP_MODE)&0x1ffffu;
    }
    unsigned elapsed = esp_cpu_get_cycle_count()-start;
    bool done = current>=n+SETTLE_SAMPLES && current<CAPACITY;
    *end = current;
    REG_CLR_BIT(DUMP_CTRL, BIT(31));
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);
    esp_ipc_isr_release_other_cpu();
    return done ? elapsed : 0;
}

static bool acquire_iq(unsigned n, unsigned divider, unsigned *capture_us) {
    REG_CLR_BIT(DUMP_CTRL, BIT(31));
    for (unsigned j=0; j<CAPACITY; j++) dump[j]=SENTINEL;
    for (unsigned j=0; j<4; j++) dump[CAPACITY+j]=SENTINEL;
    prepare_rx();
    filter_apply();
    unsigned end=0;
    unsigned cycles=acquire(n, divider, &end);
    filter_restore();
    if (!cycles || end>=CAPACITY || dump[CAPACITY]!=SENTINEL) {
        reply("ERR capture_timeout\n");
        return false;
    }
    for (unsigned j=0; j<4; j++) if (dump[CAPACITY+j]!=SENTINEL) {
        reply("ERR capture_overrun\n"); return false;
    }
    for (unsigned j=0; j<n; j++) {
        unsigned w=dump[SETTLE_SAMPLES+j];
        if(w==SENTINEL) { reply("ERR capture_memory %u\n",j); return false; }
        /* I-low/Q-high, like the shared wire layout. Swapping these lanes
         * mirrors RF frequencies in both raw I/Q and snapshot spectra. */
        samples[j]=w;
    }
    unsigned elapsed=cycles/CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    *capture_us = elapsed;
    return true;
}

static bool spectrum_acquire(unsigned n, unsigned rate, const uint32_t **data, unsigned *elapsed) {
    static const unsigned dividers[]={0,1,3,5,7,9};
    bool ok = acquire_iq(n, dividers[rate], elapsed);
    *data = samples;
    return ok;
}

#include "s31_spectrum.h"

#ifdef RING_PROBE
/* Ownership bits expose interleaved words, not an independently readable
 * contiguous window. Probe both full ownership and a single released bit. */
static unsigned IRAM_ATTR ring_probe_run(unsigned divider, unsigned *last, uint32_t live[4], bool release_bank) {
    esp_ipc_isr_stall_other_cpu();taskENTER_CRITICAL(&dump_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_WRITE(DUMP_MODE,(REG_READ(DUMP_MODE)&~0x01fe0000u)|(divider<<21)|(3u<<17));
    uint32_t ctrl=(reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17);
    REG_WRITE(DUMP_CTRL,ctrl|BIT(18));REG_WRITE(DUMP_CTRL,ctrl);
    REG_WRITE(DUMP_CTRL,ctrl|BIT(31));
    unsigned start=esp_cpu_get_cycle_count(),prev=0,wraps=0;
    while(esp_cpu_get_cycle_count()-start<CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ*10000u) {
        unsigned ptr=REG_READ(DUMP_MODE)&0x1ffffu;
        if(ptr<prev)wraps++;
        prev=ptr;
    }
    if(release_bank) {
        unsigned wait_start=esp_cpu_get_cycle_count();
        for(;;) {
            unsigned at=REG_READ(DUMP_MODE)&0xffffu;
            if(at>=8192 && at<16384)break;
            if(esp_cpu_get_cycle_count()-wait_start>CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ*10000u)break;
        }
        REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0xfe000000u);
        __asm__ volatile("fence" ::: "memory");
    }
    for(unsigned j=0;j<4;j++)live[j]=dump[j];
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_CLR_BIT(DUMP_CTRL,BIT(31));REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);esp_ipc_isr_release_other_cpu();
    *last=prev;return wraps;
}
static bool ring_probe_command(const char *line) {
    unsigned rate,release=0;char extra;
    int fields=sscanf(line,"RINGPROBE %u %u %c",&rate,&release,&extra);
    if(fields<1)return false;
    if(fields>2 || rate>5 || release>1){reply("ERR args\n");return true;}
    static const unsigned divider[]={0,1,3,5,7,9};
    prepare_rx();filter_apply();unsigned last=0;uint32_t live[4];unsigned wraps=ring_probe_run(divider[rate],&last,live,release);filter_restore();
    reply("RINGPROBE %u %u %u %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " / %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\n",rate,wraps,last,live[0],live[1],live[2],live[3],dump[0],dump[1],dump[2],dump[3]);return true;
}
#endif

static bool capture(unsigned n, unsigned divider, unsigned bits) {
    unsigned elapsed;
    if (!acquire_iq(n, divider, &elapsed)) return false;
    size_t bytes = n * 4;
    uint8_t *packed = (uint8_t *)samples;
    if (bits == 8) {
        for (unsigned j = 0; j < n; j++) {
            uint32_t w = samples[j];
            packed[2*j] = (w >> 2) & 255;
            packed[2*j+1] = (w >> 12) & 255;
        }
        bytes = n * 2;
    } else if (bits == 10) {
        for (unsigned j = 0; j < n; j += 2, packed += 5) {
            uint32_t a = samples[j] & 0xfffff;
            uint32_t b = j+1 < n ? samples[j+1] & 0xfffff : 0;
            packed[0] = a; packed[1] = a >> 8; packed[2] = (a >> 16) | (b << 4);
            if (j+1 < n) { packed[3] = b >> 4; packed[4] = b >> 12; }
        }
        bytes = (n * 20 + 7) / 8;
    }
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)samples, bytes);
    char header[96];
    int length = snprintf(header, sizeof(header), "DATA %u %08"PRIx32" %u\n", n, crc, elapsed);
    return burst_serial_send(header, length) && burst_serial_send(samples, bytes);
}

static void apply_gain(void) {
    phy_pbus_debugmode();
    phy_pbus_xpd_rx_on(0);
    phy_set_rxclk_en(1);
    phy_set_txclk_en(1);
    burst_gain_mirror(hardware_agc ? -1 : (int)gain_code);
    phy_set_txclk_en(0);
    prepare_rx();
    phy_rfrx_sat_rst(hardware_agc);
    if (hardware_agc) {
        REG_WRITE(0x20107094, gain_init);
        REG_WRITE(0x2010713c, gain_threshold);
    } else {
        REG_WRITE(0x20107094,(gain_init&~0x1fcu)|(gain_code<<2));
        REG_WRITE(0x2010713c,(gain_threshold&~0x01fc0000u)|(gain_code<<18));
    }
    phy_force_rx_gain(!hardware_agc, hardware_agc ? 0 : gain_code);
}

static bool capture_rate(unsigned n, unsigned rate, unsigned format) {
    /* Hardware dividers verified by capture-duration slopes; no software decimation. */
    static const unsigned dividers[]={0,1,3,5,7,9};
    if (n<256 || n>MAX_SAMPLES || rate>=sizeof(dividers)/sizeof(dividers[0])) {
        reply("ERR capture_settings\n"); return false;
    }
    return capture(n, dividers[rate], format);
}

static void command(const char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef RING_PROBE
    if(ring_probe_command(line))return;
#endif
    if (s31_spectrum_command(line)) return;
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
    unsigned n, rate, repeats, format;
    uint64_t nonce;
    char extra;
    if (!strcmp(line, "INFO")) reply("S31SDR 6 burst 16380\n");
    else if (!strcmp(line, "CAPS")) reply("CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS SERIALLEASE DUALSERIAL TUNEEXT RX40 LPFANA GAIN HWAGC IQ8\n");
    else if (sscanf(line, "BANDWIDTH %u %c", &n, &extra)==1 &&
             (!n || (n>=RX_BANDWIDTH_MIN && n<=RX_BANDWIDTH_MAX))) {
        rx_filter=rx_bandwidth_dcap(n); reply("OK\n");
    }
    else if (!strcmp(line, "LPF AUTO")) { rx_filter=-1; reply("OK\n"); }
    else if (sscanf(line, "LPF %u %c", &n, &extra)==1 && n<=63) { rx_filter=n; reply("OK\n"); }
    else if (!strcmp(line, "LPF?")) reply("LPF %d %u %u\n", rx_filter,
        phy_i2c_readReg(0x67,1,4), phy_i2c_readReg(0x67,1,5));
    else if (!strcmp(line, "TRANSPORT?")) reply("TRANSPORT %s %u\n", burst_serial_port()==BURST_SERIAL_UART ? "UART" : "USB", burst_serial_baud());
    else if (!strcmp(line, "LIMITS?")) {
        reply("LIMITS {\"gain\":[0,%u,1],\"bandwidth\":[%u,%u,1,0],\"rates\":[80000000,40000000,20000000,10000000,8000000,4000000],\"bits\":[8,10]}\n", gain_max, RX_BANDWIDTH_MIN, RX_BANDWIDTH_MAX);
    } else if (!strcmp(line, "RANGE?")) reply(RX_TUNING_RANGE_REPLY);
    else if (sscanf(line, "SYNC %"SCNu64" %c", &nonce, &extra) == 1) reply("SYNC %"PRIu64"\n", nonce);
    else if (!strcmp(line, "RELEASE")) reply("OK\n");
    else if (!strcmp(line, "GAIN?")) reply("GAIN %s %d 0 %u %u\n",
        hardware_agc ? "HARDWARE" : "MANUAL", hardware_agc ? -1 : (int)gain_code,
        gain_max, (unsigned)((REG_READ(RX_GAIN) >> 23) & 1));
    else if (!strcmp(line, "GAIN HARDWARE")) { hardware_agc = true; apply_gain(); reply("OK\n"); }
    else if (sscanf(line, "GAIN MANUAL %u %c", &n, &extra) == 1 && n <= gain_max) {
        hardware_agc = false; gain_code = n; apply_gain(); reply("OK\n");
    } else if (sscanf(line, "FREQ %u %c", &n, &extra) == 1 && rx_frequency_valid(n)) {
        frequency_mhz=n;
        phy_set_txclk_en(1);
        burst_gain_mirror(-1);
        phy_set_txclk_en(0);
        REG_WRITE(0x20107094, gain_init);
        REG_WRITE(0x2010713c, gain_threshold);
        s31_tune(n);
        gain_init = REG_READ(0x20107094);
        gain_threshold = REG_READ(0x2010713c);
        gain_max = (REG_READ(RX_GAIN) >> 8) & 127;
        if (gain_code > gain_max) gain_code = gain_max;
        prepare_rx(); apply_gain(); reply("OK\n");
    } else if (sscanf(line, "CAP16 %u %u %c", &n, &rate, &extra) == 2) capture_rate(n, rate, 8);
    else if (sscanf(line, "CAP20 %u %u %c", &n, &rate, &extra) == 2) capture_rate(n, rate, 10);
    else if (!strncmp(line, "CAP ", 4) && sscanf(line, "CAP %u %u %c", &n, &rate, &extra) == 2) capture_rate(n, rate, 32);
    else if (sscanf(line, "RXRUN %u %u %u %u %c", &n, &rate, &repeats, &format, &extra) == 4 &&
             repeats > 0 && repeats <= 1000 && (format == 16 || format == 20)) {
        for (unsigned j = 0; j < repeats; j++) {
            if (!capture_rate(n, rate, format/2)) return;
            vTaskDelay(1);
        }
        reply("END\n");
    } else reply("ERR command\n");
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=8192,.rx_buffer_size=8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_power_domain_on();
    esp_phy_rftest_config(1);
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    s31_tune(frequency_mhz);
    phy_loopback_mode_en(0);
    phy_bb_bss_cbw40_dig(0);
    phy_bb_cbw_chan_cfg(0);
    phy_wifi_fbw_sel(0);
    prepare_rx();
    phy_pbus_debugmode();
    dump_init();
    prepare_rx();
    *(volatile unsigned *)(phy_param+164)&=~0x200u;
    phy_set_rx_gain_table(frequency_mhz,0);
    gain_init=REG_READ(0x20107094);
    gain_threshold=REG_READ(0x2010713c);
    gain_max = (REG_READ(RX_GAIN) >> 8) & 127;
    assert(gain_max < 80);
    apply_gain();
    /* USB may be unplugged when the host uses the UART bridge. */
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
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
            command(line);
        }
        /* Ownership covers the entire binary transaction. Silence releases it
         * after five seconds. */
        lease_deadline=esp_timer_get_time()+5000000;
    }
}
