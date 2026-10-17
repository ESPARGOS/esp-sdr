/* ESP32-S31 receive-only radio for the Korvo LCD analyzer. The PHY sequences
 * mirror main/targets/esp32s31/burst/receiver.c; only the transport differs. */
#include "radio.h"
#include <assert.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "rx_bandwidth.h"
#include "rx_tuning.h"
#include "dump.h"

/* RF ownership covers the complete upper 128 KiB SRAM group while the dump
 * gate is open, not only the CAPACITY words the writer fills: CPU accesses
 * anywhere in the group corrupt memory during a snapshot. Keep the whole
 * group out of the heap (the ROM data above 0x2f07f170 is excluded anyway). */
SOC_RESERVE_MEMORY_REGION(0x2f060000, 0x2f07f170, korvo_rf_dump);
_Static_assert((RADIO_SNAPSHOT_SAMPLES + SETTLE_SAMPLES) < CAPACITY, "S31 dump stop margin");
#define RX_GAIN 0x2010702cu

static const char *TAG = "radio";
static SemaphoreHandle_t lock;
static unsigned frequency_mhz = 2412;
static unsigned gain_max, gain_code = 40;
static bool hardware_agc = true;
static int rx_filter = -1;
static unsigned gain_init, gain_threshold;
static unsigned failures;

int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd; (void)name; (void)argc; (void)argv; return -1;
}
extern unsigned phy_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(int);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(int);
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

static unsigned filter_saved[2];
static void filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        filter_saved[j] = phy_i2c_readReg(0x67, 1, 4+j);
        if (rx_filter >= 0) phy_i2c_writeReg(0x67, 1, 4+j, (filter_saved[j]&~63u)|(unsigned)rx_filter);
    }
    esp_rom_delay_us(10);
}
static void filter_restore(void) {
    if (rx_filter >= 0) for (unsigned j = 0; j < 2; j++) phy_i2c_writeReg(0x67, 1, 4+j, filter_saved[j]);
}

/* The analog filter codes stay applied between snapshots; they are restored
 * to the PHY-calibrated values around every retune or recalibration. */
static bool filter_active;
static void filter_enter(void) { if (!filter_active) { filter_apply(); filter_active = true; } }
static void filter_leave(void) { if (filter_active) { filter_restore(); filter_active = false; } }

static void prepare_rx(void) {
    if (rx_recalibration_stale()) s31_tune(frequency_mhz);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    phy_rfrx_sat_rst(hardware_agc);
    phy_force_rx_gain(!hardware_agc, hardware_agc ? 0 : gain_code);
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
        REG_WRITE(0x20107094, (gain_init&~0x1fcu)|(gain_code<<2));
        REG_WRITE(0x2010713c, (gain_threshold&~0x01fc0000u)|(gain_code<<18));
    }
    phy_force_rx_gain(!hardware_agc, hardware_agc ? 0 : gain_code);
}

static void tune(unsigned mhz) {
    frequency_mhz = mhz;
    phy_set_txclk_en(1);
    burst_gain_mirror(-1);
    phy_set_txclk_en(0);
    REG_WRITE(0x20107094, gain_init);
    REG_WRITE(0x2010713c, gain_threshold);
    s31_tune(mhz);
    gain_init = REG_READ(0x20107094);
    gain_threshold = REG_READ(0x2010713c);
    gain_max = (REG_READ(RX_GAIN) >> 8) & 127;
    if (gain_code > gain_max) gain_code = gain_max;
    prepare_rx();
    apply_gain();
}

void radio_init(void) {
    lock = xSemaphoreCreateMutex();
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
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
    *(volatile unsigned *)(phy_param+164) &= ~0x200u;
    phy_set_rx_gain_table(frequency_mhz, 0);
    gain_init = REG_READ(0x20107094);
    gain_threshold = REG_READ(0x2010713c);
    gain_max = (REG_READ(RX_GAIN) >> 8) & 127;
    assert(gain_max < 80);
    apply_gain();
    filter_enter();
    ESP_LOGI(TAG, "RX ready at %u MHz, gain table maximum %u", frequency_mhz, gain_max);
}

unsigned radio_gain_max(void) { return gain_max; }

unsigned radio_rate_hz(unsigned code) {
    static const unsigned rates[RADIO_RATE_COUNT] = {80000000, 40000000, 20000000, 10000000, 8000000, 4000000};
    return code < RADIO_RATE_COUNT ? rates[code] : 0;
}

bool radio_apply(const radio_settings_t *s) {
    if (!rx_frequency_valid(s->frequency_mhz)) return false;
    if (s->bandwidth_mhz && (s->bandwidth_mhz < RX_BANDWIDTH_MIN || s->bandwidth_mhz > RX_BANDWIDTH_MAX)) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    filter_leave();
    rx_filter = rx_bandwidth_dcap(s->bandwidth_mhz);
    hardware_agc = s->agc;
    gain_code = s->gain > gain_max ? gain_max : s->gain;
    if (s->frequency_mhz != frequency_mhz) tune(s->frequency_mhz);
    else apply_gain();
    filter_enter();
    xSemaphoreGive(lock);
    return true;
}

bool radio_snapshot(unsigned n, unsigned rate, const uint32_t **words, unsigned *elapsed_us) {
    static const unsigned dividers[RADIO_RATE_COUNT] = {0, 1, 3, 5, 7, 9};
    if (n < 256 || n > RADIO_SNAPSHOT_SAMPLES || rate >= RADIO_RATE_COUNT) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    REG_CLR_BIT(DUMP_CTRL, BIT(31));
    /* The writer fills sequentially: sentinels at the end of the wanted
     * range plus the overrun guard detect short and long captures. */
    for (unsigned j = SETTLE_SAMPLES + n - 64; j < SETTLE_SAMPLES + n + 64; j++) dump[j] = SENTINEL;
    for (unsigned j = 0; j < 4; j++) dump[CAPACITY + j] = SENTINEL;
    if (rx_recalibration_stale()) {
        /* Background PHY calibration replaced the DC tables: remeasure at
         * the tuned LO with calibrated filter registers, then restore RX. */
        filter_leave();
        prepare_rx();
        filter_enter();
    }
    unsigned end = 0;
    /* The UI core must be halted while the dump gate is open: with core 0
     * running, the heap start was corrupted within seconds on the bench
     * board. Snapshots are short (0.2-4 ms), so the display barely notices. */
    unsigned cycles = dump_acquire(n, dividers[rate], &end, true);
    bool ok = cycles && end < CAPACITY && dump[CAPACITY] == SENTINEL;
    for (unsigned j = 0; ok && j < 4; j++) if (dump[CAPACITY+j] != SENTINEL) ok = false;
    for (unsigned j = n - 64; ok && j < n; j++) if (dump[SETTLE_SAMPLES+j] == SENTINEL) ok = false;
    xSemaphoreGive(lock);
    if (!ok) { failures++; return false; }
    *words = (const uint32_t *)(dump + SETTLE_SAMPLES);
    *elapsed_us = cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    return true;
}

unsigned radio_snapshot_failures(void) { return failures; }
