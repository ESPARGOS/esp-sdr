/* ESP32-C3 burst SDR over native USB Serial/JTAG and UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_event.h"
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
#include "esp_rom_sys.h"
#if CONFIG_ESP_SDR_OLED_ENABLED
#include "oled_waterfall.h"
#endif

/* Pinned C3 librftest adctrig: 64 KiB at 0x3fcb0000, usage=2,
 * allocation bit 3. Reserve the FULL 128 KiB bank and its IRAM alias:
 * allowing heap/DMA traffic in the lower half leaves holes in RF dumps. */
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);
#define IQ_WORDS       16380u
#define IQ_BUFFER      ((uint32_t *)0x3fcb0000)
#define SRAM_OWNER_REG 0x600c1020u
extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void phy_set_freq(unsigned, int);
extern unsigned rom1_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom1_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
static int rx_filter = -1;
static unsigned rx_filter_saved[2];
/* BBTOP I/Q CBW20 capacitor codes. Preserve calibration and upper bits. */
static void rx_filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        rx_filter_saved[j] = rom1_chip_i2c_readReg(0x67, 1, 4 + j);
        if (rx_filter >= 0)
            rom1_chip_i2c_writeReg(0x67, 1, 4 + j, (rx_filter_saved[j] & ~63u) | (unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if (rx_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom1_chip_i2c_writeReg(0x67, 1, 4 + j, rx_filter_saved[j]);
}
static unsigned frequency_mhz = 2412;
static bool rx_ready;
#define C3_FREQ_MIN RX_FREQ_MIN
#define C3_FREQ_MAX RX_FREQ_MAX
static bool frequency_valid(unsigned mhz) {
    return mhz >= C3_FREQ_MIN && mhz <= C3_FREQ_MAX;
}
static void tune_rx(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    rx_lo_select(false);
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) phy_set_freq(plan.mhz, plan.offset_khz);
}
#define send_bytes burst_serial_send
static void reply(const char *s) {
    (void)send_bytes(s, strlen(s));
}
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if (rx_ready && !rx_recalibration_stale()) return;
    tune_rx(frequency_mhz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    rx_ready = true;
}

#include "burst_capture.h"
static portMUX_TYPE capture_mux = portMUX_INITIALIZER_UNLOCKED;
/* The OLED loop captures without a host; keep its failures off the wire. */
static bool capture_quiet;
static void capture_error(const char *s) {
    if (!capture_quiet) reply(s);
}
static bool acquire_iq(unsigned n, unsigned divider, unsigned *capture_us) {
    if (divider != 0 && divider != 1 && divider != 6) {
        capture_error("ERR rate\n");
        return false;
    }
    prepare_rx();

    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = 0xa5a0055au;
    rx_filter_apply();
    /* Prevent task/interrupt interleaving during the bank handoff and
     * bounded snapshot (at the maximum sample count about 205 us at 80 MS/s,
     * 410 us at 40 MS/s and 1 ms at 16 MS/s). */
    taskENTER_CRITICAL(&capture_mux);
    uint32_t owner = REG_READ(SRAM_OWNER_REG);
    int64_t start = esp_timer_get_time();
    REG_WRITE(0x60033d5c, 0);
    REG_WRITE(0x60033d90, (1u << 6) | (2u << 12) | (3u << 18));
    REG_WRITE(SRAM_OWNER_REG, (owner & ~7u) | 2u | 8u);
    __asm__ volatile("fence rw,rw" ::: "memory");
    /* Like the S3: bit 15 halves the 80 MS/s dump clock, bit 16 divides it by five. */
    uint32_t ctrl = 0x80000000u | n | (divider == 1 ? 1u << 15 : divider == 6 ? 1u << 16 : 0);
    REG_WRITE(0x60033d5c, ctrl);
    REG_WRITE(0x60033d5c, ctrl | (1u << 19));
    REG_WRITE(0x60033d5c, ctrl);
    while (!(REG_READ(0x60033d5c) & (1u << 18)) && esp_timer_get_time() - start < 20000) {}
    bool done = (REG_READ(0x60033d5c) & (1u << 18)) != 0;
    uint32_t elapsed = esp_timer_get_time() - start;
    REG_WRITE(0x60033d5c, 0);
    REG_WRITE(SRAM_OWNER_REG, owner);
    __asm__ volatile("fence rw,rw" ::: "memory");
    taskEXIT_CRITICAL(&capture_mux);
    rx_filter_restore();
    if (!done) {
        capture_error("ERR capture_timeout\n");
        return false;
    }
    for (unsigned j = 0; j < n; j++) {
        if (IQ_BUFFER[j] == 0xa5a0055au) {
            capture_error("ERR capture_incomplete\n");
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

#if CONFIG_ESP_SDR_OLED_ENABLED
/* Widest analog filter (about 62 MHz) for the display's 67.5 MHz span;
 * the host's LPF/BANDWIDTH choice is untouched. */
static bool oled_acquire(unsigned n, unsigned rate, const uint32_t **data, unsigned *elapsed) {
    int saved = rx_filter;
    rx_filter = rx_bandwidth_dcap(RX_BANDWIDTH_MAX);
    capture_quiet = true;
    bool ok = spectrum_acquire(n, rate, data, elapsed);
    capture_quiet = false;
    rx_filter = saved;
    return ok;
}
static void oled_retune(unsigned mhz) {
    if (frequency_valid(mhz)) {
        frequency_mhz = mhz;
        rx_ready = false;
    }
}
static void oled_idle(bool host_active) {
    oled_waterfall_step(frequency_mhz, host_active, oled_acquire, oled_retune);
}
#endif

#include "ring_probe.h"

static bool capture(unsigned n, unsigned divider, unsigned format) {
    unsigned elapsed;
    if (!acquire_iq(n, divider, &elapsed)) return false;
    return burst_send_capture(n, format, elapsed);
}

#define BURST_ID       "C3SDR"
#define BURST_MAX_RATE 6
#include "burst_commands.h"

static void handle_command(char *line) {
    if (burst_version_command(line)) return;
    if (burst_gpio_command(line)) return;
#ifdef RING_PROBE
    if (ring_probe_command(line)) return;
#endif
#if CONFIG_ESP_SDR_OLED_ENABLED
    if (oled_waterfall_command(line)) return;
#endif
    if (spectrum_command(line, frequency_mhz, spectrum_acquire)) return;
    if (limits_command(line)) return;
    if (gain_command(line)) return;
    if (burst_common_command(line)) return;
    unsigned n;
    char extra;
    if (!strcmp(line, "CAPS")) {
        reply("CAPS VERSION GPIO SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS SERIALLEASE "
#if CONFIG_ESP_SDR_UART_ENABLED
              "DUALSERIAL "
#endif
              "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8"
#if CONFIG_ESP_SDR_OLED_ENABLED
              " OLED"
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
        snprintf(answer, sizeof(answer), "LPF %d %u %u\n", rx_filter, rom1_chip_i2c_readReg(0x67, 1, 4) & 63,
                 rom1_chip_i2c_readReg(0x67, 1, 5) & 63);
        reply(answer);
    } else
        reply("ERR command\n");
}

void app_main(void) {
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
#if CONFIG_ESP_SDR_OLED_ENABLED
    if (oled_waterfall_init(frequency_mhz)) burst_serial_set_idle(oled_idle);
#endif
    burst_serial_serve(handle_command);
}
