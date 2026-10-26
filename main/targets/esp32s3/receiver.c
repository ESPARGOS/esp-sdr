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

#include "burst_parse.h"
#include "burst_serial.h"
#include "burst_gpio.h"
#include "burst_version.h"
#include "rx_recalibration.h"
#include "rx_tuning.h"
#include "rx_lo.h"
#include "esp_rom_sys.h"
#include "ring_capture.h"
#if CONFIG_ESP_SDR_DINMETER
#include "dinmeter_ui.h"
#endif

/* Vendor S3 adctrig uses the 64 KiB aperture at 0x3fcd0000 (MAC_DUMP_USAGE=4).
 * The continuous ring also uses the two banks below it. Keep all three, in
 * both DRAM and IRAM aliases, out of the heap and static sections. */
SOC_RESERVE_MEMORY_REGION(RING_BANK_BASE, RING_BANK_END, s3_rf_dump);
#define IQ_WORDS       16380u
#define IQ_BUFFER      ((uint32_t *)0x3fcd0000)
#define SRAM_OWNER_REG 0x600c101cu
extern void adctrig(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
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
extern void set_chanfreq(unsigned, unsigned);
extern void set_rf_freq_offset(unsigned, unsigned, int);
static void s3_tune(unsigned mhz);
static int s3_fofs; /* FOFS: PLL offset in kHz, applied from the next tune */
/* Set when the knob, not the host, chose s3_fofs; a host FREQ clears it. */
static bool fofs_from_ui;
static void s3_tune(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    /* A PLL offset also requires direct tuning on Wi-Fi channel frequencies. */
    bool channel = !s3_fofs && ((mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484);
    rx_lo_select(false);
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) set_rf_freq_offset(0, plan.mhz, plan.offset_khz + s3_fofs);
}

#define S3_FREQ_MIN RX_FREQ_MIN
#define S3_FREQ_MAX RX_FREQ_MAX
static bool frequency_valid(unsigned mhz) {
    return mhz >= S3_FREQ_MIN && mhz <= S3_FREQ_MAX;
}
static unsigned frequency_mhz = 2412;
static bool rx_ready;
#ifdef S3_RF_PROBE
static unsigned rx_clock = 0;
static unsigned rx_source, rx_mode, rx_flag, rx_wide, rx_prep = 3, rx_pack, rx_agc;
#else
enum { rx_source = 0, rx_mode = 0, rx_flag = 0, rx_wide = 0, rx_prep = 3, rx_pack = 0, rx_agc = 0 };
#endif
extern void force_rx_gain(unsigned, unsigned, unsigned);
static int rx_filter = -1; /* -1 restores the PHY-calibrated automatic mode. */
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
/* Apply only around an RX snapshot; restore before any retune. */
static unsigned rx_filter_saved[2];
static void rx_filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        rx_filter_saved[j] = rom_chip_i2c_readReg(0x67, 0, 4 + j);
        if (rx_filter >= 0)
            rom_chip_i2c_writeReg(0x67, 0, 4 + j, (rx_filter_saved[j] & ~63u) | (unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if (rx_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom_chip_i2c_writeReg(0x67, 0, 4 + j, rx_filter_saved[j]);
}
/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd;
    (void)name;
    (void)argc;
    (void)argv;
    return -1;
}
#define send_bytes burst_serial_send
static void reply(const char *s) {
    (void)send_bytes(s, strlen(s));
}
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if (rx_ready && !rx_recalibration_stale()) return;
    if (rx_prep == 1) {
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        force_rx_gain(1, 55, 0);
        rx_ready = true;
        return;
    }
    if (rx_prep == 2) {
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        rx_ready = true;
        return;
    }
    s3_tune(frequency_mhz);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    rx_ready = true;
}
/* A forced index update alone can leave continuous capture using stale RX
 * state until the next tune. Apply gain changes through the same receiver
 * setup as FREQ, before acknowledging the command. */
static void gain_reconfigure(void) {
    rx_ready = false;
    prepare_rx();
}
#include "filter_probe.h"
#include "dump_probe.h"

#include "burst_capture.h"
#ifdef SAMPLE_RATE_PROBE
/* Volatile, bounded dump-clock/source investigation; excluded from releases. */
static unsigned probe_source, probe_clock, probe_adc = 4;
extern void rom_dac_rate_set(unsigned);
static bool probe_capture;
#endif
static bool capture(unsigned n, unsigned divider, unsigned format) {
    if (divider != 0 && divider != 1 && divider != 6) {
        reply("ERR rate\n");
        return false;
    }
    prepare_rx();

    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = 0xa5a0055au;
    rx_filter_apply();
#ifdef SAMPLE_RATE_PROBE
    unsigned adc_saved = rom_chip_i2c_readReg(0x66, 0, 4);
    if (probe_adc < 4) rom_chip_i2c_writeReg(0x66, 0, 4, (adc_saved & ~12u) | (probe_adc << 2));
#endif
    uint32_t owner = REG_READ(SRAM_OWNER_REG);
    int64_t start = esp_timer_get_time();
    REG_WRITE(0x60033d5c, 0);
    REG_WRITE(0x60033d90, rx_pack | ((rx_pack + 1) << 6) | ((rx_pack + 2) << 12) | ((rx_pack + 3) << 18) |
                              (rx_agc << 24));
    REG_WRITE(SRAM_OWNER_REG, (owner & ~15u) | 4u);
    /* Native dump clocks verified with B210: bit15 halves 80 to 40 MS/s;
     * bit16 selects the 16 MS/s hardware path. No software resampling. */
    uint32_t rate_bits = divider == 1 ? (1u << 15) : divider == 6 ? (1u << 16) : 0;
    uint32_t ctrl =
        0x80000000u | rate_bits | (rx_wide << 17) | (rx_flag << 16) | (rx_source << 20) | (rx_mode << 28) | n;
#ifdef S3_RF_PROBE
    ctrl |= rx_clock << 15;
#endif
#ifdef SAMPLE_RATE_PROBE
    if (probe_capture) ctrl = (ctrl & ~0x7ff38000u) | (probe_source << 20) | (probe_clock << 15);
#endif
    REG_WRITE(0x60033d5c, ctrl);
    REG_WRITE(0x60033d5c, ctrl | (1u << 19));
    REG_WRITE(0x60033d5c, ctrl);
    while (!(REG_READ(0x60033d5c) & (1u << 18)) && esp_timer_get_time() - start < 20000) {}
    bool done = (REG_READ(0x60033d5c) & (1u << 18)) != 0;
    uint32_t elapsed = esp_timer_get_time() - start;
    REG_WRITE(0x60033d5c, 0);
    REG_WRITE(SRAM_OWNER_REG, owner);
    rx_filter_restore();
#ifdef SAMPLE_RATE_PROBE
    if (probe_adc < 4) rom_chip_i2c_writeReg(0x66, 0, 4, adc_saved);
#endif
    if (!done) {
        reply("ERR capture_timeout\n");
        return false;
    }
    for (unsigned j = 0; j < n; j++) {
        if (IQ_BUFFER[j] == 0xa5a0055au) {
            reply("ERR capture_timeout\n");
            return false;
        }
    }
    return burst_send_capture(n, format, elapsed);
}

/* Continuous modes (ring_capture.c). Reports end with one text line:
 * <TAG> status detail units pairs elapsed_us late_max work_max_cycles
 *       frames drops abandoned ffts stopped_by_host */
static void ring_report(const char *tag, const ring_result_t *r) {
    char h[224];
    snprintf(h, sizeof(h),
             "%s %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32
             " %" PRIu32 " %" PRIu32 " %" PRIu32 " %u\n",
             tag, r->status, r->detail, r->units, r->pairs, r->elapsed_us, r->late_max, r->work_max,
             r->frames, r->drops, r->abandoned, r->ffts, (unsigned)r->stopped_by_host);
    reply(h);
}
/* RINGCAP payload: "RINGDATA units rate_hz n0 n1 n2 crc32\n", then the units'
 * raw 32-bit IQ words back to back (gapless), then the RINGCAP report. */
static void ring_send_capture(const ring_result_t *r, unsigned rate) {
    uint32_t crc = 0;
    for (unsigned u = 0; u < r->units; u++) {
        const uint32_t *p = ring_capture_bank(r->cap[u].bank);
        unsigned first = r->cap[u].first, n = r->cap[u].count, head = RING_PAIRS - first;
        if (head > n) head = n;
        crc = esp_rom_crc32_le(crc, (const uint8_t *)(p + first), head * 4);
        crc = esp_rom_crc32_le(crc, (const uint8_t *)p, (n - head) * 4);
    }
    char h[128];
    snprintf(h, sizeof(h), "RINGDATA %" PRIu32 " %u %" PRIu32 " %" PRIu32 " %" PRIu32 " %08" PRIx32 "\n",
             r->units, ring_capture_rate_hz(rate), r->cap[0].count, r->units > 1 ? r->cap[1].count : 0,
             r->units > 2 ? r->cap[2].count : 0, crc);
    reply(h);
    for (unsigned u = 0; u < r->units; u++) {
        const uint32_t *p = ring_capture_bank(r->cap[u].bank);
        unsigned first = r->cap[u].first, n = r->cap[u].count, head = RING_PAIRS - first;
        if (head > n) head = n;
        (void)send_bytes(p + first, head * 4);
        if (n > head) (void)send_bytes(p, (n - head) * 4);
    }
}
static bool ring_command(const char *line) {
    unsigned ms, rate, stride, upf, mode, units, nfft, sflag = 0, n;
    char extra;
    int k;
    if (!strcmp(line, "DUAL?")) {
        char h[48];
        snprintf(h, sizeof(h), "DUAL %u %u\n",
                 (unsigned)ring_capture_dual_active() * (ring_capture_assist ? 1u : 2u),
                 (unsigned)ring_capture_core1_alive());
        reply(h);
        return true;
    }
    if (!strncmp(line, "FOFS ", 5)) {
        /* +-1 MHz covers every frequency between whole-MHz FREQ steps. */
        int off;
        if (!burst_parse_int(line + 5, -1000, 1000, &off)) {
            reply("ERR args\n");
            return true;
        }
        s3_fofs = off;
        fofs_from_ui = false;
        rx_ready = false;
        prepare_rx();
        reply("OK\n");
        return true;
    }
    if (!strcmp(line, "DC?")) {
        char h[16];
        snprintf(h, sizeof(h), "DC %u\n", ring_capture_dc_mode);
        reply(h);
        return true;
    }
    if (sscanf(line, "DC %u %c", &n, &extra) == 1 && n < 2) {
        ring_capture_dc_mode = n;
        reply("OK\n");
        return true;
    }
    if (!strcmp(line, "ASSIST?")) {
        char h[48];
        snprintf(h, sizeof(h), "ASSIST %u %" PRIu32 "\n", (unsigned)ring_capture_assist,
                 ring_capture_c0_blocks);
        reply(h);
        return true;
    }
    if (sscanf(line, "DUAL %u %c", &n, &extra) == 1 && n < 3) {
        ring_capture_set_dual(n != 0);
        ring_capture_assist = n == 1;
        reply("OK\n");
        return true;
    }
    ring_config_t c = {0};
    const char *tag;
    bool usb = burst_serial_port() == BURST_SERIAL_USB;
    if (!strcmp(line, "SPECINFO?")) {
        /* Worker profiles include 512 bins; retain the staged fallback when
         * the worker is unavailable or explicitly disabled. */
        if (ring_capture_dual_active())
            reply(
                "SPECINFO {\"continuous\":true,\"transports\":[\"USB\"],\"profiles\":["
                "[16000000,6,256,2,1],[16000000,6,512,2,2],[16000000,6,1024,2,4],[16000000,6,2048,3,7],"
                "[40000000,1,256,5,3],[40000000,1,512,5,5],[40000000,1,1024,5,9],[40000000,1,2048,7,17],"
                "[80000000,0,256,10,6],[80000000,0,512,12,10],[80000000,0,1024,14,16],[80000000,0,2048,14,30]]}\n");
        else
            reply(
                "SPECINFO {\"continuous\":true,\"transports\":[\"USB\"],\"profiles\":["
                "[16000000,6,256,4,2],[16000000,6,512,6,3],[16000000,6,1024,8,4],[16000000,6,2048,6,8],"
                "[40000000,1,256,12,4],[40000000,1,512,12,6],[40000000,1,1024,18,10],[40000000,1,2048,10,20],"
                "[80000000,0,256,48,8],[80000000,0,512,32,12],[80000000,0,1024,24,16],[80000000,0,2048,16,24]]}\n");
        return true;
    }
    if (!strcmp(line, "RINGINFO?")) {
        /* The ring reserves 192 KiB of SRAM; report what the heap kept. */
        char h[96];
        snprintf(h, sizeof(h), "RINGINFO %u %u %u %u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), RING_BANKS, RING_THRESHOLD);
        reply(h);
        return true;
    }
    if (sscanf(line, "RING %u %u %c", &ms, &rate, &extra) == 2) {
        c.mode = RING_MODE_STATS;
        c.rate = rate;
        c.duration_ms = ms;
        tag = "RING";
        if (rate != 0 && rate != 1 && rate != 6) {
            reply("ERR rate\n");
            return true;
        }
        if ((!ms && !usb) || ms > 86400000u) {
            reply("ERR args\n");
            return true;
        }
    } else if (sscanf(line, "RINGCAP %u %u %c", &units, &rate, &extra) == 2) {
        c.mode = RING_MODE_CAPTURE;
        c.rate = rate;
        c.capture_units = units;
        tag = "RINGCAP";
        if (rate != 0 && rate != 1 && rate != 6) {
            reply("ERR rate\n");
            return true;
        }
        if (!units || units > RING_BANKS) {
            reply("ERR args\n");
            return true;
        }
    } else if (!strncmp(line, "SPEC ", 5) &&
               (k = sscanf(line, "SPEC %u %u %u %u %u %u %u %c", &ms, &stride, &upf, &mode, &rate, &nfft,
                           &sflag, &extra)) >= 4 &&
               k <= 7) {
        /* Optional 5th field: rate code 6 = 16, 1 = 40, 0 = 80 Msps (default 16). */
        if (k == 4) rate = 6;
        if (k < 6) nfft = 256;
        if (!ring_capture_valid_nfft(nfft)) {
            reply("ERR nfft\n");
            return true;
        }
        if (rate != 0 && rate != 1 && rate != 6) {
            reply("ERR rate\n");
            return true;
        }
        c.mode = RING_MODE_SPEC;
        c.rate = rate;
        c.nfft = nfft;
        c.duration_ms = ms;
        c.stride = stride;
        c.units_per_frame = upf;
        c.max_hold = mode == 1;
        c.stats = k >= 7 && sflag;
        tag = "SPECEND";
        if (!usb) {
            reply("ERR transport\n");
            return true;
        }
        if (!stride || stride > 64 || !upf || upf > 1000 || mode > 1 || sflag > 1 || ms > 86400000u) {
            reply("ERR args\n");
            return true;
        }
        char h[80];
        snprintf(h, sizeof(h), "SPEC %u %u %u %u\n", nfft, ring_capture_rate_hz(rate), RING_THRESHOLD,
                 frequency_mhz);
        reply(h);
    } else if (!strncmp(line, "IQS ", 4)) {
        /* IQS ms dec bits rate [shift] [mode]: continuous decimated IQ (IQS1 frames).
         * mode 0: FIR at 0 Hz IF; mode 2: +fs/4 shift first (LO tuned fs/4 below).
         * Default shift scales the 15-bit FIR output to the requested bits. */
        unsigned f[6];
        int kk = burst_parse_uints(line + 4, f, 6);
        if (kk < 4) {
            reply("ERR args\n");
            return true;
        }
        ms = f[0];
        stride = f[1];
        nfft = f[2];
        rate = f[3];
        upf = kk >= 5 ? f[4] : nfft < 15u ? 15u - nfft : 0u;
        mode = kk >= 6 ? f[5] : 0;
        if (mode != 0 && mode != 2) {
            reply("ERR mode\n");
            return true;
        }
        if (rate != 0 && rate != 1 && rate != 6) {
            reply("ERR rate\n");
            return true;
        }
        if (!usb) {
            reply("ERR transport\n");
            return true;
        }
        if (stride < 64 || stride > 1024 || (stride & (stride - 1)) ||
            (nfft != 4 && nfft != 8 && nfft != 16) || upf > 24 || ms > 86400000u) {
            reply("ERR args\n");
            return true;
        }
        c.mode = RING_MODE_IQ;
        c.rate = rate;
        c.duration_ms = ms;
        c.iq_dec = stride;
        c.iq_bits = nfft;
        c.iq_shift = upf;
        c.iq_rot = mode == 2;
        tag = "IQSEND";
        char h[96];
        snprintf(h, sizeof(h), "IQS %u %u %u %u %u %u\n", ring_capture_rate_hz(rate), stride, nfft, upf, mode,
                 frequency_mhz);
        reply(h);
    } else
        return false;
    ring_result_t r;
    prepare_rx();
    rx_filter_apply();
    ring_capture_run(&c, &r);
    rx_filter_restore();
    if (c.mode == RING_MODE_CAPTURE && !r.status) ring_send_capture(&r, rate);
    ring_report(tag, &r);
    return true;
}

#if CONFIG_ESP_SDR_DINMETER
/* One display line: a short continuous SPEC run into the UI's sink, with
 * the UI's analog bandwidth. The host's LPF/BANDWIDTH choice is kept. */
static unsigned dinmeter_run(unsigned rate, unsigned nfft, unsigned bandwidth_mhz, unsigned ms, bool max_hold,
                             unsigned *detail) {
    ring_config_t c = {0};
    c.mode = RING_MODE_SPEC;
    c.rate = rate;
    c.nfft = nfft;
    c.duration_ms = ms;
    c.stride = 1;
    c.units_per_frame = 1000;
    c.max_hold = max_hold;
    int saved = rx_filter;
    rx_filter = rx_bandwidth_dcap(bandwidth_mhz);
    ring_result_t r;
    prepare_rx();
    rx_filter_apply();
    ring_capture_local_sink = dinmeter_sink;
    ring_capture_run(&c, &r);
    ring_capture_local_sink = NULL;
    rx_filter_restore();
    rx_filter = saved;
    *detail = r.detail;
    return r.status;
}
static void dinmeter_retune(unsigned khz) {
    unsigned mhz = khz / 1000;
    int off = (int)(khz % 1000);
    if (mhz == frequency_mhz && off == s3_fofs) return;
    frequency_mhz = mhz;
    s3_fofs = off;
    fofs_from_ui = off != 0;
    rx_ready = false;
}
static void dinmeter_idle(bool host_active) {
    dinmeter_step(frequency_mhz * 1000u + (unsigned)s3_fofs, host_active, dinmeter_run, dinmeter_retune);
}
#endif

#define BURST_ID       "S3SDR"
#define BURST_MAX_RATE 6
#include "burst_commands.h"

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef FILTER_REGISTER_PROBE
    if (filter_probe_command(line)) return;
#endif
#ifdef S3_RF_PROBE
    if (dump_probe_command(line)) return;
#endif
    if (limits_command(line)) return;
    if (gain_command(line)) return;
    if (ring_command(line)) return;
#if CONFIG_ESP_SDR_DINMETER
    if (dinmeter_command(line)) return;
    if (fofs_from_ui && !strncmp(line, "FREQ ", 5)) {
        s3_fofs = 0;
        fofs_from_ui = false;
    }
#endif
    if (burst_common_command(line)) return;
    unsigned n;
    unsigned rate __attribute__((unused)); /* only the diagnostic probes use it */
    char extra;
    if (!strcmp(line, "CAPS")) {
        reply("CAPS VERSION GPIO UARTBAUD RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
              "DUALSERIAL "
#endif
              "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8 RING SPEC SPECN SPECCAPS SPECSTAT DCT"
#if CONFIG_ESP_SDR_DINMETER
              " DINMETER"
#endif
              "\n");
    } else if (sscanf(line, "BANDWIDTH %u %c", &n, &extra) == 1 &&
               (!n || (n >= RX_BANDWIDTH_MIN && n <= RX_BANDWIDTH_MAX))) {
        rx_filter = rx_bandwidth_dcap(n);
        reply("OK\n");
    } else if (!strcmp(line, "LPF AUTO")) {
        rx_filter = -1;
        reply("OK\n");
    } else if (sscanf(line, "LPF %u %c", &n, &extra) == 1 && n <= 63) {
        rx_filter = n;
        reply("OK\n");
    } else if (!strcmp(line, "LPF?")) {
        char answer[80];
        snprintf(answer, sizeof(answer), "LPF %d %u %u\n", rx_filter, rom_chip_i2c_readReg(0x67, 0, 4) & 63,
                 rom_chip_i2c_readReg(0x67, 0, 5) & 63);
        reply(answer);
    }
#ifdef SAMPLE_RATE_PROBE
    else if (sscanf(line, "RXPROBE %u %u %c", &n, &rate, &extra) == 2 && n < 2048 && rate < 8) {
        probe_source = n;
        probe_clock = rate;
        probe_capture = true;
        capture(16380, 0, 20);
        probe_capture = false;
    } else if (sscanf(line, "ADCCLOCK %u %c", &n, &extra) == 1 && n <= 4) {
        probe_adc = n;
        reply("OK\n");
    } else if (!strcmp(line, "ADCCLOCK?")) {
        char h[64];
        snprintf(h, sizeof(h), "ADC %u\n", rom_chip_i2c_readReg(0x66, 0, 4));
        reply(h);
    }
#endif
    else
        reply("ERR command\n");
}

void app_main(void) {
#if CONFIG_ESP_SDR_DINMETER
    dinmeter_power_hold(); /* battery power stays on only while GPIO46 is high */
#endif
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 8192, .rx_buffer_size = 8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*", ESP_LOG_NONE);
    /* USB may be unplugged when the host uses the UART bridge. */
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    burst_serial_init();
    ring_capture_init();
#if CONFIG_ESP_SDR_DINMETER
    frequency_mhz = 2437;
    rx_ready = false;
    if (dinmeter_init()) {
        burst_serial_set_idle(dinmeter_idle);
        burst_serial_set_passive(dinmeter_passive_command);
    }
#endif
    burst_serial_serve(handle_command);
}
