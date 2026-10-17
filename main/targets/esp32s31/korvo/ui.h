#pragma once
#include <stdbool.h>
#include "lvgl.h"
/* Build the analyzer user interface and start the render task. */
void ui_init(lv_display_t *display);
/* Console command (SHEET n, FREQ mhz, RATE n, NFFT n, DC n, GAIN n, AGC, BW mhz,
 * MARKER x, HOLD, MAX, PAUSE, CLEAR, KEYPAD, AUTOSCALE). */
bool ui_command(const char *line);
