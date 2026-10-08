#include "sdkconfig.h"
#include "rx_recalibration.h"
#include "esp_private/phy.h"
#include <sys/lock.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

/* Count actual DC measurements, not a temperature stamp: the PHY can replace
 * a DC table without changing that stamp. Atomic loads also cover dual-core
 * receivers. The PHY mutex serializes our measurement with its tracking task. */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
/* These two Xtensa archives resolve their internal DC calls locally, so
 * --wrap cannot intercept them. Compare the exported DC tables instead.
 * Sizes are from the pinned PHY archives, not temperature-state offsets. */
#if CONFIG_IDF_TARGET_ESP32
#define RXBB_DC_BYTES 16
#define RXRF_DC_BYTES 136
#else
#define RXBB_DC_BYTES 60
#define RXRF_DC_BYTES 48
extern unsigned char phy_chan_dc[56];
static unsigned char calibrated_chan_dc[56];
#endif
extern unsigned char phy_rxbb_dc[RXBB_DC_BYTES];
extern unsigned char phy_rxrf_dc[RXRF_DC_BYTES];
static unsigned char calibrated_rxbb_dc[RXBB_DC_BYTES];
static unsigned char calibrated_rxrf_dc[RXRF_DC_BYTES];
#endif
static atomic_uint dc_generation;
static unsigned calibrated_generation;
static bool calibrated;
extern void rx_recalibrate_locked(unsigned mhz);

bool rx_recalibration_stale(void) {
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    bool stale = !calibrated ||
        memcmp(phy_rxbb_dc, calibrated_rxbb_dc, RXBB_DC_BYTES) ||
        memcmp(phy_rxrf_dc, calibrated_rxrf_dc, RXRF_DC_BYTES);
#if CONFIG_IDF_TARGET_ESP32S2
    stale = stale || memcmp(phy_chan_dc, calibrated_chan_dc, sizeof(calibrated_chan_dc));
#endif
    _lock_release(&lock);
    return stale;
#else
    return !calibrated || atomic_load(&dc_generation) != calibrated_generation;
#endif
}

void rx_recalibrate(unsigned mhz) {
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    rx_recalibrate_locked(mhz);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    memcpy(calibrated_rxbb_dc, phy_rxbb_dc, RXBB_DC_BYTES);
    memcpy(calibrated_rxrf_dc, phy_rxrf_dc, RXRF_DC_BYTES);
#if CONFIG_IDF_TARGET_ESP32S2
    memcpy(calibrated_chan_dc, phy_chan_dc, sizeof(calibrated_chan_dc));
#endif
#endif
    calibrated_generation = atomic_load(&dc_generation);
    calibrated = true;
    _lock_release(&lock);
}

/* Private, word-sized calling conventions from the pinned PHY archives.
 * Forward every register/stack argument unchanged. Wrapping the measurement
 * also covers reference-channel rebuilds outside the periodic tracker. */
#if CONFIG_IDF_TARGET_ESP32C5
extern void __real_phy_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d);
void __wrap_phy_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d) {
    __real_phy_set_rx_gain_cal_dc(a, b, c, d);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32S31
extern void __real_phy_set_rx_gain_cal_dc_new(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d);
void __wrap_phy_set_rx_gain_cal_dc_new(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d) {
    __real_phy_set_rx_gain_cal_dc_new(a, b, c, d);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32C6
extern void __real_set_rx_gain_cal_dc_new(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e);
void __wrap_set_rx_gain_cal_dc_new(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e) {
    __real_set_rx_gain_cal_dc_new(a, b, c, d, e);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32C2
extern void __real_ram_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b);
void __wrap_ram_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b) {
    __real_ram_set_rx_gain_cal_dc(a, b);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32H2
extern void __real_set_rx_gain_cal_dc(uintptr_t a);
void __wrap_set_rx_gain_cal_dc(uintptr_t a) {
    __real_set_rx_gain_cal_dc(a);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
extern void __real_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e, uintptr_t f, uintptr_t g, uintptr_t h, uintptr_t i, uintptr_t j);
void __wrap_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e, uintptr_t f, uintptr_t g, uintptr_t h, uintptr_t i, uintptr_t j) {
    __real_set_rx_gain_cal_dc(a, b, c, d, e, f, g, h, i, j);
    atomic_fetch_add(&dc_generation, 1);
}
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
/* DC table snapshots above also detect changes made by ROM-local calls. */
#else
#error RX DC tracking ABI has not been qualified for this target
#endif
