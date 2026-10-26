/* M5Stack Din Meter live spectrum + waterfall.
 *
 * Each display line is one short continuous SPEC run of the shared ring
 * capture (dual-core, gapless RF writer), redirected into dinmeter_sink()
 * instead of USB. The LCD is drawn between runs, with interrupts enabled.
 *
 * Knob: turn = tune (TUNE) or change the span (SPAN); press toggles the
 * two. Long press toggles the detector (peak hold / average). */
#include "dinmeter_ui.h"
#include "burst_serial.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define PIN_LCD_MOSI 5
#define PIN_LCD_SCLK 6
#define PIN_LCD_DC   4
#define PIN_LCD_CS   7
#define PIN_LCD_RST  8
#define PIN_LCD_BL   9
#define PIN_ENC_A    41
#define PIN_ENC_B    40
#define PIN_BUTTON   42
#define PIN_HOLD     46
#define PIN_BUZZER   3

#define W             240
#define H             135
#define STRIP         5
#define HEADER_END    18
#define AXIS_END      30
#define SPEC_END      70
#define SPEC_ROWS     (SPEC_END - AXIS_END)
#define WF_ROWS       (H - SPEC_END)
#define MAX_NFFT      2048u
#define RUN_MS        45
#define LONG_PRESS_US 700000
#define FREQ_MIN_KHZ  2210000u /* keep clear of the 1842-2210 MHz alternate LO plan */
#define FREQ_MAX_KHZ  2700000u

#ifndef CONFIG_ESP_SDR_DINMETER_COUNTS_PER_DETENT
#define CONFIG_ESP_SDR_DINMETER_COUNTS_PER_DETENT 4
#endif

typedef struct {
    uint16_t span_mhz_x10; /* displayed span */
    uint8_t rate;          /* ring rate code: 0 = 80, 1 = 40, 6 = 16 MS/s */
    uint16_t nfft;
    uint8_t bandwidth_mhz; /* analog filter */
    uint16_t step_khz;     /* tuning step per detent */
} span_t;
static const span_t spans[] = {
    {800, 0, 1024, 69, 5000}, {600, 0, 1024, 62, 5000}, {400, 1, 1024, 40, 2000}, {200, 1, 2048, 24, 1000},
    {160, 6, 1024, 14, 1000}, {100, 6, 2048, 13, 500},  {50, 6, 2048, 13, 250},   {20, 6, 2048, 13, 100},
};
#define SPAN_COUNT (sizeof(spans) / sizeof(spans[0]))
static unsigned rate_hz(unsigned code) {
    return code == 0 ? 80000000u : code == 1 ? 40000000u : 16000000u;
}

/* ---- State -------------------------------------------------------------- */

static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t strip_done;
static uint16_t *strip[2];
static pcnt_unit_handle_t encoder;
static bool present;
static const char *init_failure = "init"; /* failed init step, reported by DM? */
static esp_err_t init_error;
static unsigned run_status, run_detail, run_failures;

static unsigned span_index;
static bool knob_span; /* false: TUNE, true: SPAN */
static bool average;   /* detector: false = peak hold */

static uint8_t *bins; /* natural FFT order, 0.5 dB codes */
static unsigned frame_nfft;
static volatile bool frame_ready;
static uint8_t column[W];
static uint8_t *waterfall; /* WF_ROWS x W palette levels, ring */
static unsigned wf_head, lines;
static int floor_q4 = -1; /* 0.5 dB codes << 4 */
static int peak_offset_hz, peak_db;
static uint16_t *palette; /* 256 byte-swapped RGB565 levels */

static uint8_t *rx_frame; /* SPC1 header being received */
static unsigned rx_used;
static uint32_t frames_ok, frames_bad;

static int encoder_last, encoder_residue;
static int64_t press_start, last_detent;
static bool long_fired;

/* ---- Colours and font ---------------------------------------------------- */

static uint16_t rgb(unsigned r, unsigned g, unsigned b) {
    uint16_t c = (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
    return (uint16_t)((c >> 8) | (c << 8)); /* panel takes big-endian pixels */
}

static void build_palette(void) {
    static const uint8_t stops[][4] = {{0, 0, 0, 0},       {40, 0, 0, 110},     {90, 0, 90, 220},
                                       {130, 0, 210, 210}, {170, 40, 220, 40},  {205, 250, 230, 0},
                                       {235, 255, 60, 0},  {255, 255, 255, 255}};
    for (unsigned s = 0; s + 1 < sizeof(stops) / sizeof(stops[0]); s++)
        for (unsigned v = stops[s][0]; v <= stops[s + 1][0]; v++) {
            unsigned span = stops[s + 1][0] - stops[s][0], t = v - stops[s][0];
            unsigned c[3];
            for (unsigned k = 0; k < 3; k++)
                c[k] = (stops[s][k + 1] * (span - t) + stops[s + 1][k + 1] * t) / span;
            palette[v] = rgb(c[0], c[1], c[2]);
        }
}

/* Classic 5x7, columns LSB-top. */
static const uint8_t *glyph(char c) {
    static const uint8_t digits[10][5] = {{0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
                                          {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
                                          {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
                                          {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
                                          {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e}};
    static const uint8_t letters[26][5] = {
        {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
        {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 0x09, 0x09, 0x09, 0x01},
        {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40},
        {0x7f, 0x02, 0x0c, 0x02, 0x7f}, {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
        {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
        {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00}, colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00},
                         minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08}, plus[5] = {0x08, 0x08, 0x3e, 0x08, 0x08},
                         slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02}, blank[5] = {0};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    switch (c) {
    case '.': return dot;
    case ':': return colon;
    case '-': return minus;
    case '+': return plus;
    case '/': return slash;
    default: return blank;
    }
}

/* Draw into the strip covering rows [y0, y0 + STRIP). */
static void text(uint16_t *buf, int y0, int x, int y, int scale, uint16_t color, const char *s) {
    for (; *s; s++, x += 6 * scale) {
        const uint8_t *g = glyph(*s);
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 7; row++) {
                if (!(g[col] & (1u << row))) continue;
                for (int dy = 0; dy < scale; dy++) {
                    int py = y + row * scale + dy - y0;
                    if (py < 0 || py >= STRIP) continue;
                    for (int dx = 0; dx < scale; dx++) {
                        int px = x + col * scale + dx;
                        if (px >= 0 && px < W) buf[py * W + px] = color;
                    }
                }
            }
    }
}

static int text_width(const char *s, int scale) {
    return (int)strlen(s) * 6 * scale - scale;
}

static void fill(uint16_t *buf, int y0, int x, int y, int w, int h, uint16_t color) {
    for (int py = y; py < y + h; py++) {
        if (py < y0 || py >= y0 + STRIP) continue;
        for (int px = x; px < x + w; px++)
            if (px >= 0 && px < W) buf[(py - y0) * W + px] = color;
    }
}

/* ---- LCD --------------------------------------------------------------- */

static bool on_strip_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *event, void *ctx) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(strip_done, &woken);
    return woken == pdTRUE;
}

static bool lcd_init(void) {
    gpio_set_direction(PIN_LCD_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LCD_BL, 0);
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = W * STRIP * 2 + 16,
    };
    init_failure = "spi";
    if ((init_error = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO)) != ESP_OK) return false;
    strip_done = xSemaphoreCreateCounting(2, 2);
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40000000,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_strip_done,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    init_failure = "lcd_io";
    if ((init_error = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io)) !=
        ESP_OK)
        return false;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    init_failure = "st7789";
    if ((init_error = esp_lcd_new_panel_st7789(io, &panel_config, &panel)) != ESP_OK) return false;
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, true);
    /* 135x240 glass at controller offset (52, 40); landscape as M5GFX
     * rotation 1 (MADCTL MV|MY), or the opposite landscape when flipped. */
    esp_lcd_panel_swap_xy(panel, true);
#if CONFIG_ESP_SDR_DINMETER_LCD_FLIP
    esp_lcd_panel_mirror(panel, true, false);
    esp_lcd_panel_set_gap(panel, 40, 53);
#else
    esp_lcd_panel_mirror(panel, false, true);
    esp_lcd_panel_set_gap(panel, 40, 52);
#endif
    esp_lcd_panel_disp_on_off(panel, true);
    return true;
}

/* The waterfall and strip buffers take ~22 KiB of the S3's scarce heap. A
 * USB host may need it (IQS FIR stages allocate ~20 KiB per run), so they
 * are released while a host holds the lease and rebuilt afterwards. */
static bool buffers_acquire(void) {
    for (int k = 0; k < 2; k++)
        if (!strip[k] && !(strip[k] = heap_caps_malloc(W * STRIP * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)))
            return false;
    if (!waterfall && !(waterfall = heap_caps_calloc(WF_ROWS, W, MALLOC_CAP_INTERNAL))) return false;
    return true;
}

static void buffers_release(void) {
    for (int k = 0; k < 2; k++) {
        free(strip[k]);
        strip[k] = NULL;
    }
    free(waterfall);
    waterfall = NULL;
}

/* ---- Rendering ----------------------------------------------------------- */

static void format_mhz(char *out, size_t size, long khz, int decimals) {
    long whole = khz / 1000, frac = khz % 1000;
    if (frac < 0) frac = -frac;
    if (decimals == 3)
        snprintf(out, size, "%ld.%03ld", whole, frac);
    else if (decimals == 2)
        snprintf(out, size, "%ld.%02ld", whole, frac / 10);
    else
        snprintf(out, size, "%ld.%ld", whole, frac / 100);
}

static void format_span(char *out, size_t size, unsigned x10) {
    if (x10 % 10)
        snprintf(out, size, "%u.%uM", x10 / 10, x10 % 10);
    else
        snprintf(out, size, "%uM", x10 / 10);
}

static void render_strip(uint16_t *buf, int y0, unsigned center_khz, bool host_active) {
    const uint16_t header_bg = rgb(18, 24, 40), black = 0, white = rgb(255, 255, 255),
                   grey = rgb(110, 120, 140), grid = rgb(40, 48, 64), accent = rgb(255, 176, 0),
                   cyan = rgb(80, 220, 255), trace = rgb(255, 255, 160);
    const int floor_code = floor_q4 < 0 ? 0 : floor_q4 >> 4;
    for (int row = 0; row < STRIP; row++) {
        int y = y0 + row;
        uint16_t *line = buf + row * W;
        if (y >= H) break;
        if (y < HEADER_END) {
            for (int x = 0; x < W; x++) line[x] = header_bg;
        } else if (y < AXIS_END) {
            for (int x = 0; x < W; x++) line[x] = black;
        } else if (y < SPEC_END) {
            int level = SPEC_END - 1 - y; /* 0 at the bottom, 1.25 dB per row */
            bool grid_row = level && level % 8 == 0;
            for (int x = 0; x < W; x++) {
                int height = ((int)column[x] - floor_code + 6) * SPEC_ROWS / 100;
                if (level == height - 1 || (level == 0 && height <= 0))
                    line[x] = trace;
                else if (level < height)
                    line[x] = palette[64 + level * 3];
                else
                    line[x] = grid_row && !(x & 3) ? grid : black;
            }
        } else {
            unsigned age = (unsigned)(y - SPEC_END);
            const uint8_t *src = waterfall + ((wf_head + WF_ROWS - age) % WF_ROWS) * W;
            for (int x = 0; x < W; x++) line[x] = palette[src[x]];
        }
    }

    /* Header: centre frequency and span; the knob's target is highlighted. */
    char a[48], b[32];
    format_mhz(a, sizeof(a), center_khz, 3);
    int fw = text_width(a, 2);
    if (!knob_span) fill(buf, y0, 0, 0, fw + 6, HEADER_END, accent);
    text(buf, y0, 3, 2, 2, knob_span ? white : black, a);
    text(buf, y0, fw + 9, 9, 1, grey, "MHZ");
    format_span(b, sizeof(b), spans[span_index].span_mhz_x10);
    snprintf(a, sizeof(a), "SPAN %s", b);
    int sw = text_width(a, 2);
    if (knob_span) fill(buf, y0, W - sw - 6, 0, sw + 6, HEADER_END, accent);
    text(buf, y0, W - sw - 3, 2, 2, knob_span ? black : white, a);
    text(buf, y0, fw + 9, 1, 1, average ? cyan : accent, average ? "AVG" : "PK");

    /* Axis: edge frequencies, peak readout, ticks every tenth of the span. */
    long half = spans[span_index].span_mhz_x10 * 50L;
    int decimals = spans[span_index].span_mhz_x10 >= 100 ? 1 : 2;
    if (host_active) {
        text(buf, y0, (W - text_width("USB HOST - DISPLAY PAUSED", 1)) / 2, 19, 1, accent,
             "USB HOST - DISPLAY PAUSED");
    } else {
        format_mhz(a, sizeof(a), (long)center_khz - half, decimals);
        text(buf, y0, 0, 19, 1, grey, a);
        format_mhz(a, sizeof(a), (long)center_khz + half, decimals);
        text(buf, y0, W - text_width(a, 1), 19, 1, grey, a);
        if (lines) {
            format_mhz(b, sizeof(b), (long)center_khz + peak_offset_hz / 1000, 2);
            snprintf(a, sizeof(a), "PK %s +%dDB", b, peak_db);
            text(buf, y0, (W - text_width(a, 1)) / 2, 19, 1, cyan, a);
        }
    }
    for (int k = 0; k <= 10; k++) {
        int x = k * (W - 1) / 10;
        fill(buf, y0, x, k % 5 ? 28 : 27, 1, k % 5 ? 2 : 3, k == 5 ? accent : grey);
    }
}

static void render(unsigned center_khz, bool host_active) {
    for (int y0 = 0, k = 0; y0 < H; y0 += STRIP, k ^= 1) {
        xSemaphoreTake(strip_done, portMAX_DELAY);
        render_strip(strip[k], y0, center_khz, host_active);
        int y1 = y0 + STRIP < H ? y0 + STRIP : H;
        esp_lcd_panel_draw_bitmap(panel, 0, y0, W, y1, strip[k]);
    }
    /* No LCD DMA may be in flight when the next ring run masks interrupts. */
    xSemaphoreTake(strip_done, portMAX_DELAY);
    xSemaphoreTake(strip_done, portMAX_DELAY);
    xSemaphoreGive(strip_done);
    xSemaphoreGive(strip_done);
}

/* ---- Spectrum line ------------------------------------------------------- */

/* ESP-WebSDR displays conj(I+jQ): display bin d holds FFT bin (n/2-d) mod n. */
static uint8_t display_bin(unsigned n, unsigned d) {
    return bins[(n / 2 - d) & (n - 1)];
}

static void finish_line(void) {
    const span_t *s = &spans[span_index];
    unsigned n = frame_nfft, rate = rate_hz(s->rate);
    unsigned shown = (unsigned)((uint64_t)n * s->span_mhz_x10 * 100000u / rate);
    unsigned first = n / 2 - shown / 2, dc = n / 2;
    unsigned histogram[256] = {0}, best = 0, best_d = dc;
    for (unsigned x = 0; x < W; x++) {
        unsigned d0 = first + x * shown / W, d1 = first + (x + 1) * shown / W;
        if (d1 <= d0) d1 = d0 + 1;
        unsigned value = 0;
        bool any = false;
        for (unsigned d = d0; d < d1; d++) {
            if (d + 1 >= dc && d <= dc + 1) continue; /* zero-IF DC / LO leakage */
            unsigned v = display_bin(n, d);
            if (v > value) value = v;
            any = true;
            if (v > best) {
                best = v;
                best_d = d;
            }
        }
        if (!any) {
            unsigned l = display_bin(n, dc - 2), r = display_bin(n, dc + 2);
            value = l > r ? l : r;
        }
        column[x] = value;
        histogram[value]++;
    }
    unsigned count = 0, percentile = 0;
    while (percentile < 255 && (count += histogram[percentile]) < W / 5) percentile++;
    if (floor_q4 < 0)
        floor_q4 = percentile << 4;
    else
        floor_q4 += ((int)(percentile << 4) - floor_q4) / 3;
    int floor_code = floor_q4 >> 4;
    peak_offset_hz = (int)(((int64_t)best_d - (int64_t)dc) * rate / n);
    peak_db = ((int)best - floor_code) / 2;
    if (peak_db < 0) peak_db = 0;

    wf_head = (wf_head + 1) % WF_ROWS;
    uint8_t *row = waterfall + wf_head * W;
    for (unsigned x = 0; x < W; x++) {
        int level = ((int)column[x] - floor_code - 4) * 255 / 80; /* 40 dB */
        row[x] = level < 0 ? 0 : level > 255 ? 255 : (uint8_t)level;
    }
    lines++;
}

/* Runs inside the ring's bank-switch loop (at most 64 bytes per call, about
 * 25 us of slack at 80 MS/s), so it does constant work per byte: bins are
 * max-merged as they arrive. The stream never leaves SRAM, so the frame CRC
 * is not recomputed here. */
void dinmeter_sink(const uint8_t *data, unsigned size) {
    for (; size; data++, size--) {
        if (rx_used < 28) {
            rx_frame[rx_used++] = *data;
            if (rx_used == 28 && (memcmp(rx_frame, "SPC1", 4) || rx_frame[26] < 8 || rx_frame[26] > 11)) {
                memmove(rx_frame, rx_frame + 1, --rx_used); /* resynchronise */
                frames_bad++;
            }
            continue;
        }
        unsigned n = 1u << rx_frame[26], k = rx_used - 28;
        if (k < n && *data > bins[k]) bins[k] = *data;
        if (++rx_used == 28 + n + 4) {
            frame_nfft = n;
            frame_ready = true;
            frames_ok++;
            rx_used = 0;
        }
    }
}

/* ---- Controls ------------------------------------------------------------- */

static void reset_display_history(void) {
    memset(waterfall, 0, WF_ROWS * W);
    memset(column, 0, sizeof(column));
    floor_q4 = -1;
}

static void poll_controls(unsigned center_khz, dinmeter_retune_fn retune) {
    int64_t now = esp_timer_get_time();
    int count = 0;
    pcnt_unit_get_count(encoder, &count);
    int delta = count - encoder_last;
    encoder_last = count;
    if (count > 4000 || count < -4000) {
        pcnt_unit_clear_count(encoder);
        encoder_last = 0;
    }
#if CONFIG_ESP_SDR_DINMETER_ENCODER_REVERSE
    delta = -delta;
#endif
    encoder_residue += delta;
    int detents = encoder_residue / CONFIG_ESP_SDR_DINMETER_COUNTS_PER_DETENT;
    encoder_residue -= detents * CONFIG_ESP_SDR_DINMETER_COUNTS_PER_DETENT;
    if (detents) {
        if (knob_span) {
            int next = (int)span_index - detents; /* clockwise zooms in */
            next = next < 0 ? 0 : next >= (int)SPAN_COUNT ? (int)SPAN_COUNT - 1 : next;
            if ((unsigned)next != span_index) {
                span_index = next;
                reset_display_history();
            }
        } else {
            /* Fast spinning moves five steps per detent. */
            int step = spans[span_index].step_khz * (now - last_detent < 60000 ? 5 : 1);
            int64_t khz = (int64_t)center_khz / step * step + (int64_t)detents * step;
            if (khz < FREQ_MIN_KHZ) khz = FREQ_MIN_KHZ;
            if (khz > FREQ_MAX_KHZ) khz = FREQ_MAX_KHZ;
            retune((unsigned)khz);
        }
        last_detent = now;
    }

    bool down = gpio_get_level(PIN_BUTTON) == 0;
    if (down && !press_start) {
        press_start = now;
        long_fired = false;
    } else if (down && !long_fired && now - press_start >= LONG_PRESS_US) {
        long_fired = true;
        average = !average;
        reset_display_history();
    } else if (!down && press_start) {
        if (!long_fired && now - press_start > 30000) knob_span = !knob_span;
        press_start = 0;
    }
}

/* ---- Public ---------------------------------------------------------------- */

void dinmeter_power_hold(void) {
    gpio_set_direction(PIN_HOLD, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_HOLD, 1);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);
}

static bool encoder_init(void) {
    pcnt_unit_config_t unit_config = {.high_limit = 8000, .low_limit = -8000};
    init_failure = "encoder";
    if ((init_error = pcnt_new_unit(&unit_config, &encoder)) != ESP_OK) return false;
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    pcnt_unit_set_glitch_filter(encoder, &filter);
    pcnt_chan_config_t a_config = {.edge_gpio_num = PIN_ENC_A, .level_gpio_num = PIN_ENC_B};
    pcnt_chan_config_t b_config = {.edge_gpio_num = PIN_ENC_B, .level_gpio_num = PIN_ENC_A};
    pcnt_channel_handle_t a, b;
    if (pcnt_new_channel(encoder, &a_config, &a) != ESP_OK ||
        pcnt_new_channel(encoder, &b_config, &b) != ESP_OK)
        return false;
    pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    gpio_pullup_en(PIN_ENC_A);
    gpio_pullup_en(PIN_ENC_B);
    pcnt_unit_enable(encoder);
    pcnt_unit_clear_count(encoder);
    pcnt_unit_start(encoder);
    return true;
}

bool dinmeter_init(void) {
    gpio_config_t button = {
        .pin_bit_mask = 1ull << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&button);
    /* Heap, not BSS: static data must end below the RF ring (sram_guard.ld). */
    palette = heap_caps_malloc(256 * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    bins = heap_caps_calloc(1, MAX_NFFT, MALLOC_CAP_INTERNAL);
    rx_frame = heap_caps_malloc(28, MALLOC_CAP_INTERNAL);
    init_failure = "memory";
    if (!palette || !bins || !rx_frame) return false;
    build_palette();
    if (!encoder_init() || !lcd_init()) return false;
    init_failure = "frame_memory";
    if (!buffers_acquire()) return false;
    present = true;
    init_failure = NULL;
    render(0, false);
    gpio_set_level(PIN_LCD_BL, 1);
    return true;
}

void dinmeter_step(unsigned center_khz, bool host_active, dinmeter_run_fn run, dinmeter_retune_fn retune) {
    static bool host_shown;
    if (!present) return;
    if (host_active) {
        if (!host_shown) {
            if (strip[0] && strip[1] && waterfall) render(center_khz, true);
            buffers_release();
            host_shown = true;
        }
        return;
    }
    if (host_shown) {
        if (!buffers_acquire()) {
            buffers_release();
            return;
        } /* retry next pass */
        reset_display_history();
        host_shown = false;
    }
    poll_controls(center_khz, retune);
    const span_t *s = &spans[span_index];
    frame_ready = false;
    rx_used = 0;
    memset(bins, 0, MAX_NFFT);
    run_status = run(s->rate, s->nfft, s->bandwidth_mhz, RUN_MS, !average, &run_detail);
    if (run_status) run_failures++;
    if (!frame_ready) return;
    finish_line();
    render(center_khz, false);
}

bool dinmeter_passive_command(const char *line) {
    if (strcmp(line, "DMWF")) return false;
    /* Header, then WF_ROWS lines of W hex palette levels, newest line first. */
    char reply[2 * W + 2];
    if (!waterfall) {
        burst_serial_send("DMWF 0 0\n", 9);
        return true;
    }
    int n = snprintf(reply, sizeof(reply), "DMWF %u %u lines=%u floor=%d\n", WF_ROWS, W, lines, floor_q4 >> 4);
    burst_serial_send(reply, n);
    static const char hex[] = "0123456789abcdef";
    for (unsigned age = 0; age < WF_ROWS; age++) {
        const uint8_t *src = waterfall + ((wf_head + WF_ROWS - age) % WF_ROWS) * W;
        for (unsigned x = 0; x < W; x++) {
            reply[2 * x] = hex[src[x] >> 4];
            reply[2 * x + 1] = hex[src[x] & 15];
        }
        reply[2 * W] = '\n';
        burst_serial_send(reply, 2 * W + 1);
    }
    return true;
}

bool dinmeter_command(const char *line) {
    char reply[256];
    if (!strcmp(line, "DM?")) {
        int count = 0;
        if (encoder) pcnt_unit_get_count(encoder, &count);
        int used =
            present ? snprintf(reply, sizeof(reply), "DM PRESENT")
                    : snprintf(reply, sizeof(reply), "DM ABSENT %s 0x%x", init_failure, (unsigned)init_error);
        snprintf(reply + used, sizeof(reply) - used,
                 " span=%u knob=%s det=%s lines=%u frames=%u bad=%u run=%u/%u fails=%u floor=%d"
                 " peak=%d/%d heap=%u/%u enc=%d btn=%d\n",
                 spans[span_index].span_mhz_x10, knob_span ? "SPAN" : "TUNE", average ? "AVG" : "PK", lines,
                 (unsigned)frames_ok, (unsigned)frames_bad, run_status, run_detail, run_failures,
                 floor_q4 >> 4, peak_offset_hz, peak_db,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), count,
                 gpio_get_level(PIN_BUTTON));
        burst_serial_send(reply, strlen(reply));
        return true;
    }
    if (!strcmp(line, "DMLINE")) {
        /* Latest column levels (0.5 dB codes), for bench checks. */
        snprintf(reply, sizeof(reply), "DMLINE %d", floor_q4 >> 4);
        burst_serial_send(reply, strlen(reply));
        for (unsigned x = 0; x < W; x++) {
            snprintf(reply, sizeof(reply), " %u", column[x]);
            burst_serial_send(reply, strlen(reply));
        }
        burst_serial_send("\n", 1);
        return true;
    }
    return false;
}
