/* Stand-alone spectrum/waterfall on the ESP32-C3 SuperMini 0.42" OLED.
 * Runs from the receiver's idle loop whenever no serial host holds the
 * lease, so the burst protocol and ESP-WebSDR keep working unchanged. */
#pragma once
#include <stdbool.h>
#include "spectrum.h"

typedef void (*oled_retune_fn)(unsigned mhz);

bool oled_waterfall_init(unsigned frequency_mhz);
/* One bounded slice of work (one RF snapshot, its FFTs and at most one
 * display refresh). host_active pauses RF use and shows a status banner. */
void oled_waterfall_step(unsigned frequency_mhz, bool host_active, spectrum_acquire_fn acquire,
                         oled_retune_fn retune);
/* OLED?, OLED ON|OFF, OLEDMODE <0..2>. Replies via burst_serial_send. */
bool oled_waterfall_command(const char *line);
