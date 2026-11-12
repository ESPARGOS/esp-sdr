#pragma once
#include "rx_tuning.h"
#define SRAM_OWNER_REG 0x60095004u
#define IQ_WORDS       16380u
#define IQ_BUFFER      ((uint32_t *)0x40830000)
#define BURST_ID       "C5SDR"
/* C5 ownership bits cover whole 128 KiB banks. Reserve bank 1 and the
 * lower 64 KiB of bank 2 as dump destinations. Never write bank 2's upper
 * half: it has live heap/ROM data. continuous.c supplies an LP-SRAM stack
 * while the ring engine temporarily makes that entire bank inaccessible. */
SOC_RESERVE_MEMORY_REGION(0x40820000, 0x40850000, c5_rf_dump);
extern void adctrig(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
static bool stock_capture(unsigned n, unsigned divider) {
    adctrig(n - 1, 0, 0, divider * 2, 0, 0, 0, 0, 0);
    return true; /* Common sentinel check detects incomplete stock captures. */
}
static bool frequency_valid(unsigned mhz) {
    return rx_frequency_valid(mhz);
}

#include "tuning.h"
