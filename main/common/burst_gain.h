/* Shared snapshot RX gain control. Codes are PHY gain indices, not dB.
 * Hardware AGC controls gain unless a manual index is selected.
 * AGCPWR_CTRL7 bits 8..14 hold the maximum calibrated low-table index.
 * The pinned PHY implementations populate this field when tuning. */
#if CONFIG_IDF_TARGET_ESP32H2
extern void force_rx_gain(unsigned, unsigned);
extern void bt_rx_force(unsigned);
#else
extern void force_rx_gain(unsigned, unsigned, unsigned);
#endif
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#define BURST_GAIN_REG 0x600a702cu
#elif CONFIG_IDF_TARGET_ESP32H2
#define BURST_GAIN_REG 0x600a2840u
#elif CONFIG_IDF_TARGET_ESP32C2
#define BURST_GAIN_REG 0x6004a02cu
#else
#define BURST_GAIN_REG 0x6001c02cu
#endif
#include "burst_gain_min.h"
typedef enum { GAIN_MANUAL, GAIN_HARDWARE } burst_gain_mode_t;
static burst_gain_mode_t gain_mode = GAIN_HARDWARE;
static unsigned gain_code = 40;
static unsigned gain_max(void) {
#if CONFIG_IDF_TARGET_ESP32H2
    /* Pinned H2 set_rx_gain_table stores the last calibrated index here. */
    extern uint8_t phy_param[];
    unsigned maximum = phy_param[83];
#else
    unsigned maximum = (REG_READ(BURST_GAIN_REG) >> 8) & 127u;
#endif
#if CONFIG_IDF_TARGET_ESP32H2
    /* Above 56 the receiver saturates on its own DC offset, and the top index
     * (66 on the tested board) collapses to almost no signal. */
    return maximum < 80u ? (maximum > 56u ? 56u : maximum) : 0u;
#elif CONFIG_IDF_TARGET_ESP32C5
    return maximum < 90u ? maximum : 0u;
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
    return maximum <= 82u ? maximum : 0u;
#else
    return maximum < 80u ? maximum : 0u;
#endif /* Never expose uncalibrated slots. */
}
#if CONFIG_IDF_TARGET_ESP32C61
extern void burst_gain_mirror(int index);
extern void phy_rfrx_sat_rst(unsigned);
static unsigned gain_init_saved, gain_threshold_saved;
static bool gain_defaults_saved;
#endif
#if CONFIG_IDF_TARGET_ESP32S3
extern void burst_gain_pin_s3(int index);
#endif
static void gain_apply(void) {
    bool manual = gain_mode == GAIN_MANUAL;
    if (gain_code > gain_max()) gain_code = gain_max();
#if CONFIG_IDF_TARGET_ESP32S3
    burst_gain_pin_s3(manual ? (int)gain_code : -1);
#endif
#if CONFIG_IDF_TARGET_ESP32C61
    if (!gain_defaults_saved) {
        gain_init_saved = REG_READ(0x600a7094u);
        gain_threshold_saved = REG_READ(0x600a713cu);
        gain_defaults_saved = true;
    }
    burst_gain_mirror(manual ? (int)gain_code : -1);
    phy_rfrx_sat_rst(manual ? 0u : 1u);
    REG_WRITE(0x600a7094u, manual ? (gain_init_saved & ~0x1fcu) | (gain_code << 2) : gain_init_saved);
    REG_WRITE(0x600a713cu,
              manual ? (gain_threshold_saved & ~0x1fc0000u) | (gain_code << 18) : gain_threshold_saved);
#endif
#if CONFIG_IDF_TARGET_ESP32H2
    force_rx_gain(manual, gain_code);
    /* Keep the Bluetooth receiver running even when releasing forced gain. */
    bt_rx_force(1);
#else
    force_rx_gain(manual, gain_code, 0);
#endif
}
#if CONFIG_IDF_TARGET_ESP32S3
/* The S3 backend restarts its continuous RX path when gain changes. */
static void gain_reconfigure(void);
#endif
static bool gain_command(const char *line) {
    unsigned code;
    char extra;
    if (!strcmp(line, "GAIN?")) {
        char h[64];
        snprintf(h, sizeof(h), "GAIN %s %d 0 %u %u\n", gain_mode == GAIN_HARDWARE ? "HARDWARE" : "MANUAL",
                 gain_mode == GAIN_HARDWARE ? -1 : (int)gain_code, gain_max(),
                 (unsigned)((REG_READ(BURST_GAIN_REG) >> 23) & 1));
        reply(h);
        return true;
    }
    if (!strcmp(line, "GAIN HARDWARE")) {
        gain_mode = GAIN_HARDWARE;
    } else if (sscanf(line, "GAIN MANUAL %u %c", &code, &extra) == 1 &&
               code - GAIN_MIN <= gain_max() - GAIN_MIN) { /* GAIN_MIN..gain_max() */
        gain_mode = GAIN_MANUAL;
        gain_code = code;
    } else
        return false;
#if CONFIG_IDF_TARGET_ESP32S3
    gain_reconfigure();
#else
    gain_apply();
#endif
    reply("OK\n");
    return true;
}
