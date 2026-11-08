/* C3/S3 manual gain: verify analog PBUS settings, not the forced-index field.
 * The latter is also copied into dump samples even when the gain did not latch.
 * These ABIs and gain-word fields belong to the pinned C3/S3 PHY/ROM. */
#include "sdkconfig.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/lock.h>
#include "esp_attr.h"
#include "esp_private/phy.h"
#include "esp_rom_sys.h"
#include "soc/soc.h"
#include "rx_gain_force.h"

#if CONFIG_IDF_TARGET_ESP32S3
#define WIFI_GAINS 83u
extern void rom_rx_gain_force(unsigned, unsigned);
#define gain_set rom_rx_gain_force
#elif CONFIG_IDF_TARGET_ESP32C3
#define WIFI_GAINS 80u
extern void force_rx_gain(unsigned, unsigned, unsigned);
static void gain_set(unsigned manual, unsigned index) {
    force_rx_gain(manual, index, 0);
}
#else
#error Gain readback has only been qualified for C3/S3
#endif
#define GAIN_WRITE_SLOT 11u
typedef void (*gain_write_t)(uint32_t, uint32_t, uint32_t);
extern void **g_phyFuns;
extern void __real_set_rx_gain_table(unsigned, unsigned);
static gain_write_t gain_write;
static uint16_t codes[WIFI_GAINS];
static bool valid[WIFI_GAINS];

static void IRAM_ATTR capture_gain(uint32_t low, uint32_t high, uint32_t index) {
    if (index < WIFI_GAINS) {
        /* Gain code: RF in [15:8], BB LNA in [7:3], fine gain in [2:0].
         * Its top RF bit falls outside word 0 and is always set for Wi-Fi. */
        codes[index] = (uint16_t)(low >> 17);
        valid[index] = true;
    }
    gain_write(low, high, index);
}

void __wrap_set_rx_gain_table(unsigned mhz, unsigned skip) {
    if (g_phyFuns && g_phyFuns[GAIN_WRITE_SLOT] != (void *)capture_gain) {
        gain_write = (gain_write_t)g_phyFuns[GAIN_WRITE_SLOT];
        if (gain_write) g_phyFuns[GAIN_WRITE_SLOT] = (void *)capture_gain;
    }
    __real_set_rx_gain_table(mhz, skip);
}

static bool gain_matches(uint16_t code) {
    unsigned rf = 0x180u | (code >> 8);
    /* The PBUS BB word reverses the LNA bits and moves fine gain to [8:6]. */
    unsigned bb = ((code & 7u) << 6) | ((code & 8u) << 2) | (code & 0x10u) |
                  ((code & 0x20u) >> 2) | ((code & 0x40u) >> 4) | ((code & 0x80u) >> 6) |
                  ((code >> 8) & 1u);
    return ((REG_READ(0x60006118u) >> 9) & 0x1ffu) == rf &&
           ((REG_READ(0x6000611cu) >> 18) & 0x1ffu) == bb;
}

bool rx_gain_verified_apply(bool manual, unsigned index) {
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    bool ok = !manual;
    if (!manual) {
        gain_set(0, index);
    } else if (gain_write && index < WIFI_GAINS && valid[index]) {
        for (unsigned attempt = 0; attempt < 4; attempt++) {
            rx_gain_force(gain_set, true, index);
            if (gain_matches(codes[index])) {
                ok = true;
                break;
            }
        }
    }
    if (!ok) gain_set(0, index);
    _lock_release(&lock);
    return ok;
}

bool rx_gain_verified_check(unsigned index) {
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    uint32_t forced = REG_READ(0x6001c02cu);
    bool ok = index < WIFI_GAINS && valid[index] &&
              (forced >> 23) == (1u | (index << 1)) && gain_matches(codes[index]);
    _lock_release(&lock);
    return ok || rx_gain_verified_apply(true, index);
}
