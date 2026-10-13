#pragma once

#include <stdint.h>

/* Standalone spectrum + waterfall on the LCDWiki ES3C28P (ESP32-S3, 2.8"
 * ILI9341V 320x240 SPI). Runs while no serial host holds the radio. */
#define LCD_WATERFALL_W 320
#define LCD_WATERFALL_H 240

void lcd_waterfall_init(void);
/* Render screen row y (big-endian RGB565) exactly as last drawn; for SCREEN?. */
void lcd_waterfall_render_row(int y, uint16_t *row);
/* words: raw IQ10 capture words (I in bits 0-9, Q in bits 10-19, signed). */
void lcd_waterfall_frame(const volatile uint32_t *words, unsigned n, unsigned center_mhz, unsigned rate_msps);
