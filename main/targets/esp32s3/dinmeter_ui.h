/* M5Stack Din Meter (ESP32-S3, 240x135 ST7789, rotary encoder) stand-alone
 * spectrum and waterfall. Runs while no serial host holds the lease. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* One continuous on-chip SPEC run; frames arrive through dinmeter_sink().
 * Returns the ring status (0 = OK) with its detail in *detail. */
typedef unsigned (*dinmeter_run_fn)(unsigned rate_code, unsigned nfft, unsigned bandwidth_mhz,
                                    unsigned duration_ms, bool max_hold, unsigned *detail);
typedef void (*dinmeter_retune_fn)(unsigned khz);

/* Call first in app_main: latches the battery power-hold line. */
void dinmeter_power_hold(void);
bool dinmeter_init(void);
void dinmeter_step(unsigned center_khz, bool host_active, dinmeter_run_fn run, dinmeter_retune_fn retune);
/* Ring output consumer (interrupts are masked while it runs). */
void dinmeter_sink(const uint8_t *data, unsigned size);
/* DM?, DMDUMP, ENC? */
bool dinmeter_command(const char *line);
/* DMWF: dump the waterfall's palette levels without pausing the display. */
bool dinmeter_passive_command(const char *line);
