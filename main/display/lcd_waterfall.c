/* Standalone spectrum + waterfall for the LCDWiki ES3C28P board.
 * Pins from the ES3C28P user manual schematic (fig. 3.10, 3.16). */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "lcd_waterfall.h"

#define PIN_LCD_CS   10
#define PIN_LCD_MOSI 11
#define PIN_LCD_SCK  12
#define PIN_LCD_MISO 13
#define PIN_LCD_DC   46
#define PIN_LCD_BL   45  /* High = backlight on. LCD reset is tied to CHIP_PU. */
#define LCD_PCLK_HZ  (40 * 1000 * 1000)

/* Orientation/colour knobs for the ILI9341V (driven with the ST7789 driver,
 * which shares the command set). */
#define LCD_MIRROR_X 0
#define LCD_MIRROR_Y 0
#define LCD_INVERT   1   /* IPS panels usually need INVON. */

#define W 320
#define H 240
#define STRIPE 16
#define STATUS_H 12
#define SPEC_Y STATUS_H
#define SPEC_H 64
#define LABEL_Y (SPEC_Y + SPEC_H)
#define LABEL_H 10
#define WF_Y (LABEL_Y + LABEL_H)
#define WF_H (H - WF_Y)

#define NFFT 512
#define LOG2_NFFT 9
#define SPEC_SPAN_DB 50.0f   /* spectrum panel vertical range */
#define WF_SPAN_DB 30.0f     /* waterfall colour range above floor */

_Static_assert(H % STRIPE == 0, "stripes must tile the screen");

static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t trans_done;
static uint16_t *stripe_buf[2];
static unsigned inflight, next_buf;

static float win[NFFT], tw_re[NFFT / 2], tw_im[NFFT / 2], fre[NFFT], fim[NFFT], psd[NFFT];
static uint16_t bitrev[NFFT];
static float col_db[W], hold_db[W], sorted_db[W];
static uint8_t col_y[W], hold_y[W];
static uint8_t *wf;           /* WF_H x W ring of colour indices */
static unsigned wf_head;      /* row holding the newest line */
static uint16_t cmap[256];    /* byte-swapped RGB565 */
static float floor_db;
static bool have_floor;
static float fps;
static int64_t last_frame_us;
static char status[64];
static unsigned label_center, label_rate;

static uint16_t rgb(unsigned r, unsigned g, unsigned b) {
    uint16_t c = ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
    return (uint16_t)((c >> 8) | (c << 8));
}

/* 5x7 glyphs, column-major, LSB at top. */
static const char glyph_chars[] = " +-./0123456789:ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const uint8_t glyphs[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x08,0x08,0x3E,0x08,0x08},{0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
    {0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
};
_Static_assert(sizeof(glyphs) / 5 == sizeof(glyph_chars) - 1, "glyph table mismatch");

static void text_row(uint16_t *row, int gy, int x, const char *s, uint16_t color) {
    if (gy < 0 || gy > 6) return;
    for (; *s; s++, x += 6) {
        const char *p = strchr(glyph_chars, *s);
        assert(p && "glyph missing from font");
        const uint8_t *g = glyphs[p - glyph_chars];
        for (int c = 0; c < 5; c++) {
            if (x + c >= 0 && x + c < W && (g[c] >> gy) & 1) row[x + c] = color;
        }
    }
}

static bool IRAM_ATTR on_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ev, void *ctx) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(trans_done, &woken);
    return woken == pdTRUE;
}

static uint16_t *acquire_stripe(void) {
    if (inflight == 2) {
        xSemaphoreTake(trans_done, portMAX_DELAY);
        inflight--;
    }
    return stripe_buf[next_buf];
}

static void push_stripe(int y0) {
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y0, W, y0 + STRIPE, stripe_buf[next_buf]));
    inflight++;
    next_buf ^= 1;
}

static void drain(void) {
    while (inflight) {
        xSemaphoreTake(trans_done, portMAX_DELAY);
        inflight--;
    }
}

static void build_cmap(void) {
    /* viridis anchors */
    static const float a[][3] = {{68, 1, 84}, {59, 82, 139}, {33, 145, 140}, {94, 201, 98}, {253, 231, 37}};
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f * 4;
        int k = t >= 4 ? 3 : (int)t;
        float f = t - k;
        cmap[i] = rgb((unsigned)(a[k][0] + f * (a[k + 1][0] - a[k][0])),
                      (unsigned)(a[k][1] + f * (a[k + 1][1] - a[k][1])),
                      (unsigned)(a[k][2] + f * (a[k + 1][2] - a[k][2])));
    }
}

void lcd_waterfall_init(void) {
    gpio_config_t bl = {.pin_bit_mask = 1ULL << PIN_LCD_BL, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&bl));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LCD_BL, 0));

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI, .miso_io_num = PIN_LCD_MISO, .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = W * STRIPE * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    trans_done = xSemaphoreCreateCounting(2, 0);
    assert(trans_done);
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS, .dc_gpio_num = PIN_LCD_DC, .spi_mode = 0,
        .pclk_hz = LCD_PCLK_HZ, .trans_queue_depth = 4, .on_color_trans_done = on_trans_done,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));
    esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = -1, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR, .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &dev, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, LCD_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y));

    for (int i = 0; i < 2; i++) {
        stripe_buf[i] = heap_caps_malloc(W * STRIPE * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(stripe_buf[i]);
    }
    wf = heap_caps_calloc(WF_H * W, 1, MALLOC_CAP_INTERNAL);
    assert(wf);

    for (int i = 0; i < NFFT; i++) {
        win[i] = 0.5f - 0.5f * cosf(2 * (float)M_PI * i / NFFT);
        unsigned r = 0;
        for (int b = 0; b < LOG2_NFFT; b++) r |= ((i >> b) & 1) << (LOG2_NFFT - 1 - b);
        bitrev[i] = r;
    }
    for (int i = 0; i < NFFT / 2; i++) {
        tw_re[i] = cosf(2 * (float)M_PI * i / NFFT);
        tw_im[i] = -sinf(2 * (float)M_PI * i / NFFT);
    }
    build_cmap();

    for (int y = 0; y < H; y += STRIPE) {
        memset(acquire_stripe(), 0, W * STRIPE * 2);
        push_stripe(y);
    }
    drain();
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LCD_BL, 1));
}

static void fft(void) {
    for (int len = 2, step = NFFT / 2; len <= NFFT; len <<= 1, step >>= 1) {
        int half = len / 2;
        for (int i = 0; i < NFFT; i += len) {
            for (int j = 0; j < half; j++) {
                float wr = tw_re[j * step], wi = tw_im[j * step];
                int a = i + j, b = a + half;
                float xr = fre[b] * wr - fim[b] * wi, xi = fre[b] * wi + fim[b] * wr;
                fre[b] = fre[a] - xr; fim[b] = fim[a] - xi;
                fre[a] += xr; fim[a] += xi;
            }
        }
    }
}

static int cmp_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static void compute(const volatile uint32_t *words, unsigned n) {
    unsigned nseg = n / NFFT;
    assert(nseg > 0);
    memset(psd, 0, sizeof(psd));
    for (unsigned s = 0; s < nseg; s++) {
        const volatile uint32_t *w = words + s * NFFT;
        float mi = 0, mq = 0;
        for (int k = 0; k < NFFT; k++) {
            uint32_t v = w[k];
            float i = (float)((int32_t)(v << 22) >> 22), q = (float)((int32_t)(v << 12) >> 22);
            fre[bitrev[k]] = i; fim[bitrev[k]] = q;
            mi += i; mq += q;
        }
        mi /= NFFT; mq /= NFFT;
        for (int k = 0; k < NFFT; k++) {
            float wk = win[bitrev[k]];
            fre[k] = (fre[k] - mi) * wk; fim[k] = (fim[k] - mq) * wk;
        }
        fft();
        for (int k = 0; k < NFFT; k++) psd[k] += fre[k] * fre[k] + fim[k] * fim[k];
    }
    /* fftshift and reduce NFFT bins to W columns by max */
    for (int c = 0; c < W; c++) {
        int b0 = c * NFFT / W, b1 = (c + 1) * NFFT / W;
        float m = 0;
        for (int b = b0; b < b1; b++) {
            float p = psd[(b + NFFT / 2) % NFFT];
            if (p > m) m = p;
        }
        col_db[c] = 10 * log10f(m / nseg + 1e-3f);
    }
    memcpy(sorted_db, col_db, sizeof(col_db));
    qsort(sorted_db, W, sizeof(float), cmp_float);
    float f = sorted_db[W / 5];   /* 20th percentile ~ noise floor */
    floor_db = have_floor ? 0.9f * floor_db + 0.1f * f : f;
    if (!have_floor) for (int c = 0; c < W; c++) hold_db[c] = col_db[c];
    have_floor = true;

    float top = floor_db - 5 + SPEC_SPAN_DB;
    uint8_t *row = wf + wf_head * W;
    for (int c = 0; c < W; c++) {
        hold_db[c] = fmaxf(col_db[c], hold_db[c] - 0.3f);
        float v = (col_db[c] - floor_db + 3) / WF_SPAN_DB * 255;
        row[c] = v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v;
        float y = (top - col_db[c]) / SPEC_SPAN_DB * (SPEC_H - 1);
        col_y[c] = y < 0 ? 0 : y > SPEC_H - 1 ? SPEC_H - 1 : (uint8_t)y;
        y = (top - hold_db[c]) / SPEC_SPAN_DB * (SPEC_H - 1);
        hold_y[c] = y < 0 ? 0 : y > SPEC_H - 1 ? SPEC_H - 1 : (uint8_t)y;
    }
}

/* Column of a frequency offset (MHz from centre); -1 if off screen. */
static int freq_col(int off_mhz, unsigned rate) {
    int c = (int)lroundf(W / 2 + (float)off_mhz * W / rate - 0.5f);
    return c >= 0 && c < W ? c : -1;
}

static unsigned tick_step(unsigned rate) { return rate >= 80 ? 10 : rate >= 40 ? 5 : 2; }

static void render_row(int y, uint16_t *row) {
    static const uint16_t black = 0;
    uint16_t grid = rgb(48, 48, 48), trace = rgb(255, 255, 160), fill = rgb(20, 70, 90),
             hold = rgb(255, 120, 40), text = rgb(220, 220, 220);
    if (y < STATUS_H) {
        for (int x = 0; x < W; x++) row[x] = black;
        text_row(row, y - 2, 2, status, text);
    } else if (y < LABEL_Y) {
        int sy = y - SPEC_Y;
        bool hgrid = sy % (SPEC_H / 5) == 0;
        for (int x = 0; x < W; x++) row[x] = hgrid ? grid : black;
        int step = tick_step(label_rate), lim = label_rate / 2;
        for (int f = -lim / step * step; f <= lim; f += step) {
            int c = freq_col(f, label_rate);
            if (c >= 0) row[c] = grid;
        }
        for (int x = 0; x < W; x++) {
            int y0 = col_y[x], y1 = x ? col_y[x - 1] : y0;
            int lo = y0 < y1 ? y0 : y1, hi = y0 < y1 ? y1 : y0;
            if (sy > y0) row[x] = fill;
            if (sy >= lo && sy <= hi) row[x] = trace;
            if (sy == hold_y[x]) row[x] = hold;
        }
    } else if (y < WF_Y) {
        for (int x = 0; x < W; x++) row[x] = black;
        int step = tick_step(label_rate) * 2, lim = label_rate / 2;
        for (int f = -lim / step * step; f <= lim; f += step) {
            int c = freq_col(f, label_rate);
            if (c < 0) continue;
            char s[8];
            snprintf(s, sizeof(s), "%u", label_center + f);
            int x = c - (int)strlen(s) * 3;
            text_row(row, y - LABEL_Y - 1, x, s, text);
        }
    } else {
        const uint8_t *src = wf + ((wf_head + WF_H - (y - WF_Y)) % WF_H) * W;
        for (int x = 0; x < W; x++) row[x] = cmap[src[x]];
    }
}

void lcd_waterfall_frame(const volatile uint32_t *words, unsigned n, unsigned center_mhz, unsigned rate_msps) {
    int64_t now = esp_timer_get_time();
    if (last_frame_us) {
        float f = 1e6f / (float)(now - last_frame_us);
        fps = fps ? 0.9f * fps + 0.1f * f : f;
    }
    last_frame_us = now;

    wf_head = (wf_head + 1) % WF_H;
    compute(words, n);
    label_center = center_mhz;
    label_rate = rate_msps;
    snprintf(status, sizeof(status), "%u MHZ  %u MS/S  FLOOR %d DB  %.1f FPS",
             center_mhz, rate_msps, (int)floor_db, fps);

    for (int y0 = 0; y0 < H; y0 += STRIPE) {
        uint16_t *buf = acquire_stripe();
        for (int r = 0; r < STRIPE; r++) render_row(y0 + r, buf + r * W);
        push_stripe(y0);
    }
    drain();
}
