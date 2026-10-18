/* Modified 2026-10-18: wire the shared continuous-capture engine
 * (main/common/ring_capture.c) to this family receiver, which already compiles
 * and runs it but had no export; the only prior wiring lives in
 * main/targets/esp32s3/receiver.c. Original upstream source and license
 * attribution remain.
 */
/* Shared ESP32-C5/C6/C61 burst receiver. Chip differences: target-selected chip.h. */
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
extern void phy_chip_set_chan(unsigned, unsigned);
extern void phy_rx_filter_mode(unsigned);
static unsigned frequency_mhz = 2412;
static bool rx_ready;
#if CONFIG_IDF_TARGET_ESP32C5
static unsigned rx_channel_mode;
#endif
#if !CONFIG_IDF_TARGET_ESP32C6
static int rx_filter = -1; /* -1 restores the PHY-calibrated automatic mode. */
#endif
static int rx_analog_filter = -1;
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#define RX_FILTER_REG 4u
#else
#define RX_FILTER_REG 6u
#endif
extern unsigned phy_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
/* RX-only analog capacitance. Preserve PHY calibration between snapshots,
 * across retunes; zero is the widest tested code, not bypass. */
static void rx_analog_apply(unsigned saved[2]) {
    for (unsigned j = 0; j < 2; j++) {
        saved[j] = phy_chip_i2c_readReg(0x67, 1, RX_FILTER_REG + j);
        if (rx_analog_filter >= 0)
            phy_i2c_writeReg(0x67, 1, RX_FILTER_REG + j, (saved[j] & ~63u) | (unsigned)rx_analog_filter);
    }
}
static void rx_analog_restore(const unsigned saved[2]) {
    if (rx_analog_filter >= 0)
        for (unsigned j = 0; j < 2; j++) phy_i2c_writeReg(0x67, 1, RX_FILTER_REG + j, saved[j]);
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
    static unsigned calibrated_mhz;
    if (rx_ready && !rx_recalibration_stale()) return;
#if CONFIG_IDF_TARGET_ESP32C61
    burst_gain_mirror(-1);
    if (gain_defaults_saved) {
        REG_WRITE(0x600a7094u, gain_init_saved);
        REG_WRITE(0x600a713cu, gain_threshold_saved);
        gain_defaults_saved = false;
    }
#endif
    if (calibrated_mhz != frequency_mhz || rx_recalibration_stale()) {
        rx_recalibrate(frequency_mhz);
        calibrated_mhz = frequency_mhz;
    }
#if CONFIG_IDF_TARGET_ESP32C5
    phy_chip_set_chan(frequency_mhz, rx_channel_mode);
#else
    phy_chip_set_chan(frequency_mhz, 0);
#endif
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
#if !CONFIG_IDF_TARGET_ESP32C6
    if (rx_filter >= 0) phy_rx_filter_mode((unsigned)rx_filter);
#endif
    gain_apply();
#if CONFIG_IDF_TARGET_ESP32C6
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
#endif
    rx_ready = true;
}
#include "filter_probe.h"

#include "burst_capture.h"
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
/* Volatile, bounded dump-clock/source investigation; excluded from releases. */
static unsigned probe_source, probe_clock, probe_adc = 4;
extern void phy_adc_rate_set(unsigned);
extern unsigned phy_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
static bool probe_capture;
#endif
static bool acquire_iq(unsigned n, unsigned divider, unsigned *capture_us) {
#if CONFIG_IDF_TARGET_ESP32C6
    if (divider != 0 && divider != 1 && divider != 6) {
        reply("ERR rate\n");
        return false;
    }
#endif
    prepare_rx();

    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = 0xa5a0055au;
    for (unsigned j = 0; j < 4; j++) IQ_BUFFER[n + j] = 0x5a5aa5a5u ^ j;
    unsigned analog_saved[2];
    rx_analog_apply(analog_saved);
    uint32_t owner = REG_READ(SRAM_OWNER_REG);
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    unsigned adc_saved = phy_chip_i2c_readReg(0x66, 0, 4);
    uint32_t adc_digital_saved = REG_READ(0x600a0448);
    if (probe_adc < 2) phy_adc_rate_set(probe_adc);
#endif
    bool done = true;
    int64_t start = esp_timer_get_time();
    /* Vendor selector 0 maps to raw source 15 and pulses the software trigger.
     * In particular, do NOT set CTRL bit 17 as in the C61 continuous backend:
     * on this C5 it produced only a short, incomplete snapshot. */
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    if (probe_capture && probe_source > 0) {
        /* Seed stock packing/clock setup, then bounded raw-source capture. */
        adctrig(255, 0, 0, probe_clock * 2u, 0, 0, 0, 0, 0);
        for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = 0xa5a0055au;
        uint32_t mode = REG_READ(0x600a9008);
        REG_WRITE(0x600a9008, (mode & ~0x001e0000u) | ((probe_source - 1) << 17));
        REG_WRITE(SRAM_OWNER_REG, (owner & ~0xf00u) | 0x10200u);
        uint32_t ctrl = 0x80000000u | n;
        REG_WRITE(0x600a9004, ctrl);
        REG_WRITE(0x600a9004, ctrl | (1u << 19));
        REG_WRITE(0x600a9004, ctrl);
        int64_t wait_start = esp_timer_get_time();
        while (!(REG_READ(0x600a9004) & (1u << 22)) && esp_timer_get_time() - wait_start < 20000) {}
        REG_WRITE(0x600a9004, 0);
        REG_WRITE(0x600a9008, mode);
    } else
        done = stock_capture(n, probe_capture ? probe_clock : divider);
#else
    done = stock_capture(n, divider);
#endif
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    if (probe_adc < 2) {
        phy_i2c_writeReg(0x66, 0, 4, adc_saved);
        REG_WRITE(0x600a0448, adc_digital_saved);
    }
#endif
    REG_WRITE(SRAM_OWNER_REG, owner);
    rx_analog_restore(analog_saved);
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
    for (unsigned j = 0; j < 4; j++) {
        if (IQ_BUFFER[n + j] != (0x5a5aa5a5u ^ j)) {
            reply("ERR capture_overrun\n");
            return false;
        }
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

static bool capture(unsigned n, unsigned divider, unsigned format) {
    unsigned elapsed;
    if (!acquire_iq(n, divider, &elapsed)) return false;
    return burst_send_capture(n, format, elapsed);
}

#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
/* Continuous capture export (ring_capture.c). Reports end with one text line:
 * <TAG> status detail units pairs elapsed_us late_max work_max_cycles
 *       frames drops abandoned ffts stopped_by_host */
static void ring_report(const char *tag, const ring_result_t *r) {
    char h[224];
    snprintf(h, sizeof(h),
             "%s %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu64
             " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %u\n",
             tag, r->status, r->detail, r->units, r->pairs, r->elapsed_us, r->late_max,
             r->work_max, r->frames, r->drops, r->abandoned, r->ffts, (unsigned)r->stopped_by_host);
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
    snprintf(h, sizeof(h),
             "RINGDATA %" PRIu32 " %u %" PRIu32 " %" PRIu32 " %" PRIu32 " %08" PRIx32 "\n",
             r->units, ring_capture_rate_hz(rate), r->cap[0].count,
             r->units > 1 ? r->cap[1].count : (uint32_t)0, (uint32_t)0, crc);
    reply(h);
    for (unsigned u = 0; u < r->units; u++) {
        const uint32_t *p = ring_capture_bank(r->cap[u].bank);
        unsigned first = r->cap[u].first, n = r->cap[u].count, head = RING_PAIRS - first;
        if (head > n) head = n;
        (void)send_bytes(p + first, head * 4);
        if (n > head) (void)send_bytes(p, (n - head) * 4);
    }
}

static bool ring_export_command(const char *line) {
    unsigned units, rate;
    char extra;
    if (!strcmp(line, "RINGINFO?")) {
        /* The ring banks are reserved in SRAM; report what the heap kept. */
        char h[96];
        snprintf(h, sizeof(h), "RINGINFO %u %u %u %u\n",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), RING_BANKS, RING_THRESHOLD);
        reply(h);
        return true;
    }
    if (sscanf(line, "RINGCAP %u %u %c", &units, &rate, &extra) != 2) return false;
    if (!ring_capture_rate_hz(rate)) {
        reply("ERR rate\n");
        return true;
    }
    if (!units || units > RING_BANKS) {
        reply("ERR args\n");
        return true;
    }
    ring_config_t cfg = {.mode = RING_MODE_CAPTURE, .rate = rate, .capture_units = units};
    prepare_rx();
    unsigned saved[2];
    rx_analog_apply(saved);
    ring_result_t r;
    ring_capture_run(&cfg, &r);
    rx_analog_restore(saved);
    if (!r.status) ring_send_capture(&r, rate);
    ring_report("RINGCAP", &r);
    return true;
}
#endif

#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
static bool ring_test(const char *line) {
    /* RINGINFO?/RINGCAP: raw-IQ export of the engine above. */
    if (ring_export_command(line)) return true;
    unsigned ms, rate, stride = 1, upf = 1, det = 0, n = 256, stats = 0;
    char extra;
    int fields =
        sscanf(line, "SPEC %u %u %u %u %u %u %u %c", &ms, &stride, &upf, &det, &rate, &n, &stats, &extra);
    bool spec = fields == 6 || fields == 7;
    if (spec && (n != 256 || burst_serial_port() == BURST_SERIAL_UART)) return false;
#ifdef RING_PROBE
    if (!spec)
        spec =
            sscanf(line, "RINGSPEC %u %u %u %u %u %u %c", &ms, &stride, &upf, &det, &rate, &n, &extra) == 6;
    if (!spec && sscanf(line, "RINGTEST %u %u %c", &ms, &rate, &extra) != 2) return false;
#else
    if (!spec) return false;
#endif
    if (ms > 86400000u || !ring_capture_rate_hz(rate) || !stride || !upf || det > 1 || stats > 1 ||
        stride > 64 || upf > 1000 || n != 256) {
        reply("ERR spec_args\n");
        return true;
    }
    if (spec) {
        ring_capture_init();
        char h[80];
        snprintf(h, sizeof(h), "SPEC %u %u %u %u\n", n, ring_capture_rate_hz(rate), RING_THRESHOLD,
                 frequency_mhz);
        reply(h);
    }
    prepare_rx();
    unsigned saved[2];
    rx_analog_apply(saved);
    ring_config_t cfg = {.mode = spec ? RING_MODE_SPEC : RING_MODE_STATS,
                         .rate = rate,
                         .duration_ms = ms,
                         .nfft = n,
                         .stride = stride,
                         .units_per_frame = upf,
                         .max_hold = det == 1,
                         .stats = stats != 0};
    ring_result_t r;
    ring_capture_run(&cfg, &r);
    rx_analog_restore(saved);
    char h[200];
    snprintf(h, sizeof(h), "%s %u %u %u %llu %llu %u %u %u %u %u %u %u\n", spec ? "SPECEND" : "RINGTEST",
             (unsigned)r.status, (unsigned)r.detail, (unsigned)r.units, (unsigned long long)r.pairs,
             (unsigned long long)r.elapsed_us, (unsigned)r.late_max, (unsigned)r.work_max, (unsigned)r.frames,
             (unsigned)r.drops, (unsigned)r.abandoned, (unsigned)r.ffts, r.stopped_by_host);
    reply(h);
    return true;
}
#endif

static void handle_command(char *line);

void app_main(void) {

    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    usb_serial_jtag_driver_config_t usb = {.tx_buffer_size = 512, .rx_buffer_size = 512};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    cfg.static_rx_buf_num = 4;
#endif
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*", ESP_LOG_NONE);
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    burst_serial_init();
    burst_serial_serve(handle_command);
}

#define BURST_MAX_RATE (CONFIG_IDF_TARGET_ESP32C6 ? 6 : 5)
#include "burst_commands.h"

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef RING_PROBE
    if (ring_probe_command(line)) return;
#endif
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    if (ring_test(line)) return;
#endif
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
#ifdef FILTER_REGISTER_PROBE
    if (filter_probe_command(line)) return;
#endif
    if (limits_command(line)) return;
    if (gain_command(line)) return;
    if (burst_common_command(line)) return;
    unsigned n;
    unsigned rate __attribute__((unused)); /* only the diagnostic probes use it */
    char extra;
    if (!strcmp(line, "CAPS")) {
        reply(
            "CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS GAIN HWAGC IQ8 SERIALLEASE"
            " TUNEEXT"
#if CONFIG_IDF_TARGET_ESP32C6
            " RX40 RX16"
#else
            " LPF LPF12"
#endif
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
            /* Advertised exactly where ring_capture.c is compiled, which is
             * also where these commands are answered. IQS stays absent:
             * RING_MODE_IQ is ESP32-S3 only. */
            " RINGINFO RINGCAP"
#endif
            " ALPF"
#if CONFIG_ESP_SDR_UART_ENABLED
            " DUALSERIAL"
#endif
            "\n");
    }
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    else if (sscanf(line, "BANDWIDTH %u %c", &n, &extra) == 1 &&
             (!n || (n >= RX_BANDWIDTH_MIN && n <= RX_BANDWIDTH_MAX))) {
        rx_analog_filter = rx_bandwidth_dcap(n);
#if CONFIG_IDF_TARGET_ESP32C5
        unsigned mode = rx_bandwidth_phy_mode(n);
        if (mode != rx_channel_mode) {
            rx_channel_mode = mode;
            rx_ready = false;
            prepare_rx();
        }
#endif
        reply("OK\n");
    }
#endif
    else if (!strcmp(line, "ALPF AUTO")) {
        rx_analog_filter = -1;
        reply("OK\n");
    } else if (sscanf(line, "ALPF %u %c", &n, &extra) == 1 && n <= 63) {
        rx_analog_filter = (int)n;
        reply("OK\n");
    } else if (!strcmp(line, "ALPF?")) {
        prepare_rx();
        char answer[64];
        snprintf(answer, sizeof(answer), "ALPF %d %u %u\n", rx_analog_filter,
                 phy_chip_i2c_readReg(0x67, 1, RX_FILTER_REG),
                 phy_chip_i2c_readReg(0x67, 1, RX_FILTER_REG + 1));
        reply(answer);
    }
#if !CONFIG_IDF_TARGET_ESP32C6
    else if (!strcmp(line, "LPF AUTO")) {
        rx_filter = -1;
        rx_ready = false;
        prepare_rx();
        reply("OK\n");
    } else if (sscanf(line, "LPF %u %c", &n, &extra) == 1 && (n == 0 || n == 4 || n == 8 || n == 12)) {
        rx_filter = (int)n;
        rx_ready = false;
        prepare_rx();
        reply("OK\n");
    } else if (!strcmp(line, "LPF?")) {
        char answer[64];
        snprintf(answer, sizeof(answer), "LPF %d %u\n", rx_filter,
                 (unsigned)((REG_READ(0x600a0430) >> 18) & 15));
        reply(answer);
    }
#endif
#if defined(C5_TUNE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    else if (sscanf(line, "FREQEX %u %c", &n, &extra) == 1 && n >= 100 && n <= 7500) {
        frequency_mhz = n;
        rx_ready = false;
        prepare_rx();
        reply("OK\n");
    }
#endif
#if defined(SAMPLE_RATE_PROBE) && CONFIG_IDF_TARGET_ESP32C5
    else if (sscanf(line, "RXPROBE %u %u %c", &n, &rate, &extra) == 2 && n < 17 && rate < 8) {
        probe_source = n;
        probe_clock = rate;
        probe_capture = true;
        capture(16380, 0, 20);
        probe_capture = false;
    } else if (sscanf(line, "ADCCLOCK %u %c", &n, &extra) == 1 && (n < 2 || n == 4)) {
        probe_adc = n;
        reply("OK\n");
    }
#endif
    else
        reply("ERR command\n");
}
