#include "sdkconfig.h"
#include "rx_recalibration.h"
#include <stdint.h>
#include "soc/soc.h"

/* These private ABIs and offsets belong to the PHY archives pinned in
 * firmware-targets.json. A table rebuild alone retains the previous DC
 * measurements (for example, state bit 0x80 on the newer Wi-Fi PHYs).
 *
 * Calibration normally tunes fixed reference channels internally. Restrict
 * the frequency override to this synchronous measurement interval; boot and
 * ordinary channel setup continue to use the original functions. */
static unsigned measurement_mhz;
extern unsigned char phy_param[];

#if !CONFIG_IDF_TARGET_ESP32H2
static void release_manual_gain(void) {
#if CONFIG_IDF_TARGET_ESP32S31
    const unsigned reg = 0x2010702cu;
#elif CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    const unsigned reg = 0x600a702cu;
#elif CONFIG_IDF_TARGET_ESP32C2
    const unsigned reg = 0x6004a02cu;
#elif CONFIG_IDF_TARGET_ESP32
    const unsigned reg = 0x3ff5c02cu;
#else
    const unsigned reg = 0x6001c02cu;
#endif
    /* Previous manual gain must not override the measurement's gain sweep. */
    REG_CLR_BIT(reg, 1u << 23);
}
#endif

#if CONFIG_IDF_TARGET_ESP32C5
extern void __real_phy_chip_set_chan_ana(unsigned);
void __wrap_phy_chip_set_chan_ana(unsigned mhz) {
    __real_phy_chip_set_chan_ana(measurement_mhz ? measurement_mhz : mhz);
}
extern void phy_set_rx_gain_cal_dc(unsigned, unsigned, void *, void *);
extern void phy_adc_rate_cal_rxdc(void);
extern void phy_set_rx_gain_table(unsigned, unsigned);
void rx_recalibrate_locked(unsigned mhz) {
    measurement_mhz = mhz;
    release_manual_gain();
    __real_phy_chip_set_chan_ana(mhz);
    unsigned band = phy_param[42];
    uint32_t *flags = (void *)(phy_param + 148);
    /* Retain the PHY's installed IQ coefficients. Loopback IQ calibration
     * emits a tone through the antenna. Refresh DC without that TX step. */
    *flags &= ~0x280u;
    if (band) {
        /* Fill every interpolation bin with fresh measurements at this LO. */
        phy_set_rx_gain_cal_dc(0, 1, phy_param + 384, phy_param + 912);
    } else {
        phy_set_rx_gain_cal_dc(0, 1, phy_param + 312, phy_param + 896);
        phy_adc_rate_cal_rxdc();
        phy_set_rx_gain_cal_dc(1, 1, phy_param + 348, phy_param + 904);
    }
    *flags |= 0x480u;
    phy_set_rx_gain_table(mhz, 0);
    measurement_mhz = 0;
}
#elif CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32S31
#include "esp_rom_sys.h"
#include "rx_pll_cal.h"
extern void __real_phy_set_channel_rfpll_freq(unsigned, unsigned, unsigned);
void __wrap_phy_set_channel_rfpll_freq(unsigned mhz, unsigned crystal, unsigned mode) {
    __real_phy_set_channel_rfpll_freq(measurement_mhz ? measurement_mhz : mhz, crystal, mode);
    if (measurement_mhz) (void)rx_pll_recover(measurement_mhz);
}
extern void phy_set_rx_gain_table(unsigned, unsigned);
void rx_recalibrate_locked(unsigned mhz) {
    measurement_mhz = mhz;
    release_manual_gain();
    uint32_t *flags = (void *)(phy_param + 164);
    /* Preserve the startup IQ coefficients; expire only DC/table caches. */
    *flags &= ~0x280u;
    phy_set_rx_gain_table(mhz, 0);
    measurement_mhz = 0;
}
#elif CONFIG_IDF_TARGET_ESP32H2
extern void __real_chip_v7_set_chan_ana(unsigned);
extern void phy_set_freq(unsigned, int);
void __wrap_chip_v7_set_chan_ana(unsigned channel) {
    __real_chip_v7_set_chan_ana(channel);
    if (measurement_mhz) phy_set_freq(measurement_mhz, 0);
}
extern void set_rx_gain_cal_iq(void *, unsigned);
extern void force_rx_gain(unsigned, unsigned);
static uint16_t startup_iq[4];
static unsigned startup_iq_valid;
extern void __real_write_gain_mem(uint32_t, uint32_t, unsigned);
void __wrap_write_gain_mem(uint32_t a, uint32_t b, unsigned index) {
    if (startup_iq_valid) {
        /* H2 RF gain groups C7/D7/E7/F7 occupy word 1 bits 20..21.
         * Loopback measures a phase/amplitude pair for each group. The stock
         * efuse helper supplies only TWO BYTES at phy_param+60; keep that
         * layout intact and install the measured pair in each gain word. */
        unsigned group = (b >> 20) & 3u;
        uint16_t iq = startup_iq[group];
        b = (b & ~0x1fffu) | ((iq >> 1) & 0x1f80u) | (iq & 0x7fu);
    }
    __real_write_gain_mem(a, b, index);
}
extern void set_rx_gain_table(void);
/* Keep the four RF gain-group corrections established at startup. The single
 * efuse pair is not an equivalent replacement on this PHY. This is the existing
 * startup loopback step, now explicit and never repeated for a user retune. */
void rx_h2_calibrate_iq_at_boot(void) {
    if (startup_iq_valid) return;
    force_rx_gain(0, 0);
    phy_set_freq(2412, 0);
    uint32_t iq_control = REG_READ(0x600a0450u);
    set_rx_gain_cal_iq(startup_iq, 0);
    REG_WRITE(0x600a0450u, iq_control);
    startup_iq_valid = 1;
}
void rx_recalibrate_locked(unsigned mhz) {
    measurement_mhz = mhz;
    force_rx_gain(0, 0);
    phy_set_freq(mhz, 0);
    /* Keep startup IQ; expire only receive DC. */
    *(uint32_t *)(phy_param + 52) &= ~0x100u;
    set_rx_gain_table();
    measurement_mhz = 0;
}

#else
#include "rx_lo.h"
extern void __real_chip_v7_set_chan_ana(unsigned);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
extern void rom_set_rf_freq_offset(unsigned, unsigned, int);
#if CONFIG_IDF_TARGET_ESP32
#include "soc/rtc.h"
#else
#include "esp_rom_sys.h"
#include "pll.h"
#endif
#elif CONFIG_IDF_TARGET_ESP32S3
extern void set_rf_freq_offset(unsigned, unsigned, int);
#else
extern void phy_set_freq(unsigned, int);
#endif
static void measurement_tune(void) {
    rx_lo_plan_t plan = rx_lo_plan(measurement_mhz);
#if CONFIG_IDF_TARGET_ESP32
    unsigned crystal = rtc_clk_xtal_freq_get();
    unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
    ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
    rom_set_rf_freq_offset(crystal == 26 ? 1 : crystal == 24 ? 2 : 0,
                          plan.mhz, (plan.offset_khz * 1024 + 500) / 1000);
#elif CONFIG_IDF_TARGET_ESP32S2
    rom_set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
    (void)s2_pll_calibrate();
#elif CONFIG_IDF_TARGET_ESP32S3
    set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
#else
    phy_set_freq(plan.mhz, plan.offset_khz);
#endif
    rx_lo_select(plan.alternate);
}
void __wrap_chip_v7_set_chan_ana(unsigned channel) {
    __real_chip_v7_set_chan_ana(channel);
    if (measurement_mhz) measurement_tune();
}
#if CONFIG_IDF_TARGET_ESP32C6
extern void set_rx_gain_table(void);
#elif CONFIG_IDF_TARGET_ESP32C2
extern void set_rx_gain_table_new(unsigned, unsigned, unsigned, unsigned);
#else
extern void set_rx_gain_table(unsigned, unsigned);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
extern uint32_t chip7_sleep_params[];
#endif
#endif
void rx_recalibrate_locked(unsigned mhz) {
    measurement_mhz = mhz;
    release_manual_gain();
    measurement_tune();
#if CONFIG_IDF_TARGET_ESP32C6
    /* Invalidate DC/table state, preserving IQ-valid bit 0x400. */
    *(uint32_t *)(phy_param + 164) &= ~0x280u;
    set_rx_gain_table();
#elif CONFIG_IDF_TARGET_ESP32C2
    /* C2 uses bit 0x400 for IQ validity, like C6, at a different offset. */
    *(uint32_t *)(phy_param + 328) &= ~0x300u;
    set_rx_gain_table_new(rx_lo_plan(mhz).mhz, 0, 1, 1);
#elif CONFIG_IDF_TARGET_ESP32
    /* Rebuild BT/Wi-Fi DC and gain memory, retaining IQ-valid bit 0x400. */
    chip7_sleep_params[0] &= ~0x20320u;
    set_rx_gain_table(rx_lo_plan(mhz).mhz, 0);
#elif CONFIG_IDF_TARGET_ESP32S2
    chip7_sleep_params[0] &= ~0x20240u;
    set_rx_gain_table(rx_lo_plan(mhz).mhz, 0);
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    /* set_rx_gain_param skips the TX-tone IQ step while 0x400 is set. */
    *(uint32_t *)(phy_param + 288) &= ~0x200u;
    set_rx_gain_table(rx_lo_plan(mhz).mhz, 0);
#else
#error RX recalibration ABI has not been qualified for this target
#endif
    measurement_mhz = 0;
}
#endif
