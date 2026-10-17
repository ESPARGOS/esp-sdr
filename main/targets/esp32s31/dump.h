/* ESP32-S31 TCM ADC-dump snapshot engine, shared by the burst and LCD
 * applications. One receiver translation unit per firmware includes it.
 * Register sequence derived from the vendor adctrig; see burst/receiver.c
 * for the memory ownership rules of the upper 128 KiB SRAM group. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_ipc_isr.h"
#include "freertos/FreeRTOS.h"
#include "soc/soc.h"
#include "soc/hp_system_reg.h"

#define DUMP_CTRL 0x20109004u
#define DUMP_MODE 0x20109008u
#define DUMP_MAC 0x2010900cu
#define DUMP_BYTES 0x20109018u
#define CAPACITY 26624u
#define SETTLE_SAMPLES 256u
#define SENTINEL 0xa55aa55au
_Static_assert(0x2f060000u + (CAPACITY+4)*4 <= 0x2f07afb0u, "S31 dump excludes ROM memory");
static volatile uint32_t *const dump = (void *)0x2f060000;
static portMUX_TYPE dump_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned reset_ctrl;

/* No PHY/I2C, allocation, logging or scheduler calls inside the open gate.
 * The writer uses continuous aperture mode even for snapshots: the vendor's
 * finite test mode writes at TCM offset zero, overlapping the application.
 * Stop before the first wrap, retaining 256 initial samples for settling.
 * The remaining >9K words give ample margin for MMIO stop latency. */
static void IRAM_ATTR dump_init(void) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&dump_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_CLR_BIT(0x201008cc, 0x00780000);
    REG_WRITE(0x201070b8, (REG_READ(0x201070b8)&~7u)|1u);
    REG_SET_BIT(0x20100800, BIT(2));
    REG_WRITE(0x20109c04, UINT32_MAX);
    REG_CLR_BIT(DUMP_MAC, BIT(31));
    REG_SET_BIT(HP_SYSTEM_TCM_RAM_PWR_CTRL0_REG, HP_SYSTEM_REG_HP_SYSTEM_TCM_CLK_FORCE_ON);
    REG_SET_BIT(0x20109c04, BIT(31)|BIT(21));
    REG_SET_BIT(0x20109c08, BIT(31));
    REG_SET_BIT(0x20109c14, 0xe400);
    REG_WRITE(0x20109c0c, (REG_READ(0x20109c0c)&~0xff00f000u)|0x44004000u);
    REG_SET_BIT(0x20109c10, BIT(31));
    REG_CLR_BIT(0x20109c10, BIT(31));
    /* Preserve implementation-specific reset fields (observed 0x03c00000).
     * Clearing them stalls the writer, even with all dump clocks enabled. */
    reset_ctrl=REG_READ(DUMP_CTRL);
    REG_SET_BIT(0x20100800, 4);
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE)&~0x01fe0000u)|(3 << 17));
    REG_WRITE(DUMP_BYTES, (REG_READ(DUMP_BYTES)&~0xffffffu) | 24 | (25<<6) | (26<<12) | (27<<18) | BIT(24));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu) | CAPACITY | BIT(17) | BIT(18));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu) | CAPACITY | BIT(17));
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);
    esp_ipc_isr_release_other_cpu();
}

/* Capture n + SETTLE_SAMPLES words at the given hardware divider. Returns the
 * elapsed CPU cycles, or zero on timeout. Both firmwares stall the other core
 * for the whole snapshot: on the bench ESP32-S31-Korvo-1, letting the UI core
 * run while the gate was open corrupted internal RAM within seconds. Flash
 * writes must never race a snapshot: a stall request from each core would
 * deadlock inside the IPC spinlock. */
static unsigned IRAM_ATTR dump_acquire(unsigned n, unsigned divider, unsigned *end, bool stall_other_cpu) {
    if (stall_other_cpu) esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&dump_mux);
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0xff000000u);
    __asm__ volatile("fence" ::: "memory");
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE)&~0x01fe0000u)|(divider<<21)|(3u<<17));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17)|BIT(18));
    REG_WRITE(DUMP_CTRL, (reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17));
    unsigned start = esp_cpu_get_cycle_count();
    REG_SET_BIT(DUMP_CTRL, BIT(31));
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));
    unsigned current=0;
    while (current<n+SETTLE_SAMPLES && esp_cpu_get_cycle_count()-start < 3000000u) {
        current=REG_READ(DUMP_MODE)&0x1ffffu;
    }
    unsigned elapsed = esp_cpu_get_cycle_count()-start;
    bool done = current>=n+SETTLE_SAMPLES && current<CAPACITY;
    *end = current;
    REG_CLR_BIT(DUMP_CTRL, BIT(31));
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0);
    __asm__ volatile("fence" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);
    if (stall_other_cpu) esp_ipc_isr_release_other_cpu();
    return done ? elapsed : 0;
}
