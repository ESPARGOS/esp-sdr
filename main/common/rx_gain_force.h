#pragma once
#include <stdbool.h>
#include <stdint.h>

extern void esp_rom_delay_us(uint32_t us);

/* The gain-force signal crosses into the receiver clock domain. A pair of
 * back-to-back MMIO writes is not a reliable low/high pulse, especially when
 * reapplying the same index. Keep it low, then allow the selected gain to settle.
 * The caller must have stopped capture; set_gain is the target's PHY API. */
static inline void rx_gain_force(void (*set_gain)(unsigned, unsigned), bool manual, unsigned index) {
    if (manual) {
        set_gain(0, index);
        esp_rom_delay_us(2);
    }
    set_gain(manual, index);
    if (manual) esp_rom_delay_us(2);
}
