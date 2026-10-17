#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#define DISPLAY_WIDTH 800
#define DISPLAY_HEIGHT 480

/* RGB panel, GT1151 touch and the LVGL port. Returns the LVGL display. */
lv_display_t *display_init(void);
esp_lcd_panel_handle_t display_panel(void);
/* Copy one RGB565 band into the panel frame buffer. Caller holds the LVGL lock. */
void display_blit(int x0, int y0, int x1, int y1, const uint16_t *pixels);
/* Raw frame-buffer dump for the UART screenshot tool. */
const uint16_t *display_frame_buffer(void);
