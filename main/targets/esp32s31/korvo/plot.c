/* Spectrum and waterfall renderer.
 *
 * The spectrum is drawn straight into the RGB panel frame buffer (PSRAM)
 * from per-column tables, one cache write-back per frame. The waterfall is
 * a PSRAM ring image: each frame adds one row and the pixel processing
 * accelerator copies the ring into the frame buffer as two blocks, so the
 * CPU never rewrites the whole waterfall. LVGL owns the surrounding widgets
 * and only repaints this area when a popup covers it; the caller holds the
 * LVGL lock while rendering so the two never write the same rows at once. */
#include "plot.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "display.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "plot";
static plot_area_t area;
static float *trace, *maximum, *current;   /* PSRAM: touched once per frame */
static bool trace_valid, maximum_valid;
static unsigned bins;
static uint8_t *history;      /* PSRAM ring of PLOT_MAX_WATERFALL rows x PLOT_MAX_WIDTH dB codes */
static unsigned history_head, history_count;
static uint16_t *image;       /* PSRAM ring image of the waterfall, PLOT_MAX_WIDTH stride */
static unsigned image_head;   /* ring row holding the newest display row */
static float image_floor = NAN, image_range = NAN;
static ppa_client_handle_t ppa;
static uint16_t column_lo[PLOT_MAX_WIDTH], column_hi[PLOT_MAX_WIDTH];
static unsigned map_bins, map_width;
static int16_t line_top[PLOT_MAX_WIDTH], line_bottom[PLOT_MAX_WIDTH], fill_from[PLOT_MAX_WIDTH], max_y[PLOT_MAX_WIDTH];
static uint16_t column_bg[PLOT_MAX_WIDTH];
static uint16_t palette[256], lut[256];
static int64_t prof[4];
static char profile_text[96];

static inline uint16_t rgb(unsigned r, unsigned g, unsigned b) {
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
static inline uint16_t hex(uint32_t c) { return rgb(c >> 16, (c >> 8) & 255, c & 255); }
static uint16_t blend(uint32_t a, uint32_t b, float t) {
    unsigned r = (unsigned)lrintf(((a >> 16) & 255) * (1 - t) + ((b >> 16) & 255) * t);
    unsigned g = (unsigned)lrintf(((a >> 8) & 255) * (1 - t) + ((b >> 8) & 255) * t);
    unsigned bl = (unsigned)lrintf((a & 255) * (1 - t) + (b & 255) * t);
    return rgb(r, g, bl);
}
#define BG_SPECTRUM 0x182126
#define BG_WATERFALL 0x11191e
#define TRACE 0x37c964
#define MAXHOLD 0xe6b969
#define MARKER 0xffe3a0
static uint16_t c_bg, c_bgw, c_grid_h, c_grid_major, c_grid_minor, c_trace, c_fill, c_max, c_marker, c_tune;

/* Waterfall palette stops from ESP-WebSDR. */
static void build_palette(void) {
    static const uint8_t stops[8][3] = {{3,8,23},{18,31,81},{38,63,153},{30,139,181},{66,213,174},{217,237,103},{255,162,61},{255,244,228}};
    for (unsigned i = 0; i < 256; i++) {
        float p = i / 255.0f * 7, t = p - floorf(p);
        unsigned j = (unsigned)floorf(p);
        if (j > 6) { j = 6; t = 1; }
        unsigned c[3];
        for (unsigned k = 0; k < 3; k++) c[k] = (unsigned)lrintf(stops[j][k] * (1 - t) + stops[j+1][k] * t);
        palette[i] = rgb(c[0], c[1], c[2]);
    }
}

plot_ticks_t plot_ticks(double span, int width) {
    double raw = span / fmax(2.0, width / 110.0);
    double p = pow(10, floor(log10(raw))), m = raw / p;
    double step = (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
    double minor = step / (fabs(step / p - 2) < 1e-9 ? 4 : 5);
    return (plot_ticks_t){step, minor};
}

const char *plot_profile(void) { return profile_text; }

void plot_init(void) {
    build_palette();
    c_bg = hex(BG_SPECTRUM); c_bgw = hex(BG_WATERFALL);
    c_grid_h = hex(0x344047); c_grid_major = hex(0x33424a); c_grid_minor = hex(0x202a2f);
    c_trace = hex(TRACE); c_fill = blend(BG_SPECTRUM, TRACE, 0.27f); c_max = hex(MAXHOLD);
    c_marker = hex(MARKER); c_tune = blend(BG_SPECTRUM, TRACE, 0.56f);
    history = heap_caps_calloc(PLOT_MAX_WATERFALL * PLOT_MAX_WIDTH, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    image = heap_caps_aligned_calloc(128, PLOT_MAX_WATERFALL * PLOT_MAX_WIDTH, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    trace = heap_caps_calloc(3 * PLOT_MAX_BINS, sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(history && image && trace);
    maximum = trace + PLOT_MAX_BINS;
    current = maximum + PLOT_MAX_BINS;
    ppa_client_config_t client = {.oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1};
    ESP_ERROR_CHECK(ppa_register_client(&client, &ppa));
    ESP_LOGI(TAG, "waterfall history %u KiB, image %u KiB (PSRAM)", (unsigned)(PLOT_MAX_WATERFALL * PLOT_MAX_WIDTH / 1024),
             (unsigned)(PLOT_MAX_WATERFALL * PLOT_MAX_WIDTH * 2 / 1024));
}

static inline uint8_t db_code(float db) {
    float c = (db + 160.0f) * 2.0f;
    return c <= 0 ? 0 : c >= 255 ? 255 : (uint8_t)lrintf(c);
}

static void build_lut(float floor_db, float range) {
    for (unsigned code = 0; code < 256; code++) {
        float db = code / 2.0f - 160.0f, q = (db - floor_db) / range * 255.0f;
        lut[code] = palette[q <= 0 ? 0 : q >= 255 ? 255 : (unsigned)q];
    }
    image_floor = floor_db; image_range = range;
}

/* Re-render every waterfall row from the stored dB codes. */
static void image_rebuild(void) {
    unsigned Hw = area.waterfall_height, W = area.width;
    for (unsigned d = 0; d < Hw; d++) {
        uint16_t *dst = image + ((image_head + d) % Hw) * PLOT_MAX_WIDTH;
        if (d < history_count) {
            const uint8_t *src = history + ((history_head + PLOT_MAX_WATERFALL - d) % PLOT_MAX_WATERFALL) * PLOT_MAX_WIDTH;
            for (unsigned x = 0; x < W; x++) dst[x] = lut[src[x]];
        } else for (unsigned x = 0; x < W; x++) dst[x] = c_bgw;
    }
}

void plot_reset_traces(void) { trace_valid = maximum_valid = false; }
void plot_clear(void) {
    plot_reset_traces();
    history_count = 0;
    if (area.width) image_rebuild();
}
/* A new width changes the column mapping and clears the history; a new
 * height (a sheet covering part of the waterfall) only re-renders it, so
 * rows recorded while the sheet was open reappear when it closes. */
void plot_set_area(const plot_area_t *a) {
    int old_width = area.width;
    area = *a;
    if (area.width > PLOT_MAX_WIDTH) area.width = PLOT_MAX_WIDTH;
    if (area.waterfall_height > PLOT_MAX_WATERFALL) area.waterfall_height = PLOT_MAX_WATERFALL;
    image_head = 0;
    if (area.width != old_width) plot_clear();
    else image_rebuild();
}

static void ensure_map(unsigned n, unsigned w) {
    if (map_bins == n && map_width == w) return;
    for (unsigned x = 0; x < w; x++) {
        unsigned lo = (unsigned)floorf((float)x / w * n), hi = (unsigned)ceilf((float)(x + 1) / w * n);
        if (lo >= n) lo = n - 1;
        if (hi > n) hi = n;
        if (hi <= lo) hi = lo + 1;
        column_lo[x] = lo; column_hi[x] = hi;
    }
    map_bins = n; map_width = w;
}

/* Column value: maximum over the covered bins, or a linear interpolation when
 * a bin spans several columns. */
static inline float column_value(const float *v, unsigned n, unsigned w, unsigned x) {
    if (n >= w) {
        float m = v[column_lo[x]];
        for (unsigned j = column_lo[x] + 1; j < column_hi[x]; j++) if (v[j] > m) m = v[j];
        return m;
    }
    float pos = ((float)x + 0.5f) / w * n - 0.5f;
    if (pos <= 0) return v[0];
    if (pos >= n - 1) return v[n - 1];
    unsigned i = (unsigned)pos; float t = pos - i;
    return v[i] * (1 - t) + v[i + 1] * t;
}

void plot_push(const float *db, unsigned n, const plot_params_t *p) {
    if (n < 8 || n > PLOT_MAX_BINS || !area.width) return;
    if (n != bins) { bins = n; plot_reset_traces(); }
    memcpy(current, db, n * sizeof(float));
    if (p->fill_dc) {
        unsigned c = n / 2;
        float m = 0.5f * (current[c - 2] + current[c + 2]);
        current[c - 1] = current[c] = current[c + 1] = m;
    }
    if (!trace_valid || p->average <= 0) { memcpy(trace, current, n * sizeof(float)); trace_valid = true; }
    else { float a = p->average, b = 1 - a; for (unsigned j = 0; j < n; j++) trace[j] = a * trace[j] + b * current[j]; }
    if (!maximum_valid) { memcpy(maximum, current, n * sizeof(float)); maximum_valid = true; }
    else for (unsigned j = 0; j < n; j++) if (current[j] > maximum[j]) maximum[j] = current[j];
    unsigned w = area.width, Hw = area.waterfall_height;
    ensure_map(n, w);
    history_head = (history_head + 1) % PLOT_MAX_WATERFALL;
    uint8_t *row = history + history_head * PLOT_MAX_WIDTH;
    for (unsigned x = 0; x < w; x++) row[x] = db_code(column_value(current, n, w, x));
    if (history_count < PLOT_MAX_WATERFALL) history_count++;
    /* Newest row goes to the ring row above the previous newest one. */
    const float range = p->range_db > 1 ? p->range_db : 1;
    if (image_floor != p->floor_db || image_range != range) { build_lut(p->floor_db, range); image_rebuild(); return; }
    if (!Hw) return;
    image_head = (image_head + Hw - 1) % Hw;
    uint16_t *dst = image + image_head * PLOT_MAX_WIDTH;
    for (unsigned x = 0; x < w; x++) dst[x] = lut[row[x]];
}

/* Copy ring rows [first, first + count) to frame-buffer rows starting at y. */
static void image_copy(unsigned first, unsigned count, int y) {
    if (!count) return;
    ppa_srm_oper_config_t op = {
        .in = {.buffer = image, .pic_w = PLOT_MAX_WIDTH, .pic_h = PLOT_MAX_WATERFALL, .block_w = area.width, .block_h = count,
               .block_offset_x = 0, .block_offset_y = first, .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
        .out = {.buffer = (void *)display_frame_buffer(), .buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2,
                .pic_w = DISPLAY_WIDTH, .pic_h = DISPLAY_HEIGHT, .block_offset_x = area.x, .block_offset_y = y,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0, .scale_x = 1, .scale_y = 1,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    esp_err_t e = ppa_do_scale_rotate_mirror(ppa, &op);
    if (e != ESP_OK) ESP_LOGE(TAG, "PPA copy failed: %s", esp_err_to_name(e));
}

void plot_render(const plot_params_t *p) {
    const int W = area.width, Hs = area.spectrum_height, Hw = area.waterfall_height;
    if (!W || !Hs) return;
    const float range = p->range_db > 1 ? p->range_db : 1;
    prof[0] = esp_timer_get_time();
    /* Per-column tables: polyline segment, fill start, max-hold row, background. */
    if (trace_valid && bins) {
        ensure_map(bins, W);
        int prev = 0;
        for (int x = 0; x < W; x++) {
            float v = column_value(trace, bins, W, x);
            float y = (1 - (v - p->floor_db) / range) * (Hs - 1);
            int t = (int)(y < 0 ? 0 : y > Hs - 1 ? Hs - 1 : lrintf(y));
            if (!x) prev = t;
            line_top[x] = (int16_t)(prev < t ? prev : t);
            line_bottom[x] = (int16_t)((prev > t ? prev : t) + 1);
            fill_from[x] = (int16_t)(t + 2);
            float m = column_value(maximum, bins, W, x);
            float ym = (1 - (m - p->floor_db) / range) * (Hs - 1);
            max_y[x] = (int16_t)(ym < 0 ? 0 : ym > Hs - 1 ? Hs - 1 : lrintf(ym));
            prev = t;
        }
    } else for (int x = 0; x < W; x++) { line_top[x] = line_bottom[x] = fill_from[x] = max_y[x] = Hs + 2; }
    for (int x = 0; x < W; x++) column_bg[x] = c_bg;
    int tune_x = -1;
    if (p->span_hz > 0) {
        plot_ticks_t t = plot_ticks(p->span_hz, W);
        double f0 = p->center_hz - p->span_hz / 2;
        for (double f = ceil(f0 / t.minor_hz) * t.minor_hz; f <= f0 + p->span_hz; f += t.minor_hz) {
            double r = round(f / t.minor_hz) * t.minor_hz;
            int x = (int)lrint((r - f0) / p->span_hz * W);
            if (x < 0 || x >= W) continue;
            column_bg[x] = fabs(r / t.step_hz - round(r / t.step_hz)) < 1e-6 ? c_grid_major : c_grid_minor;
        }
        if (fabs(p->tune_hz - p->center_hz) > 1) {
            tune_x = (int)lrint((p->tune_hz - f0) / p->span_hz * W);
            if (tune_x < 0 || tune_x >= W) tune_x = -1;
        }
    }
    prof[1] = esp_timer_get_time();
    /* Spectrum rows straight into the frame buffer. */
    uint16_t *fb = (uint16_t *)display_frame_buffer();
    int grid_rows[5];
    for (int i = 0; i <= 4; i++) grid_rows[i] = (int)lrintf(i * (Hs - 1) / 4.0f);
    for (int y = 0; y < Hs; y++) {
        uint16_t *row = fb + (area.y + y) * DISPLAY_WIDTH + area.x;
        bool hgrid = y == grid_rows[0] || y == grid_rows[1] || y == grid_rows[2] || y == grid_rows[3] || y == grid_rows[4];
        if (hgrid) {
            for (int x = 0; x < W; x++)
                row[x] = (y >= line_top[x] && y <= line_bottom[x]) ? c_trace : y >= fill_from[x] ? c_fill : c_grid_h;
        } else {
            for (int x = 0; x < W; x++)
                row[x] = (y >= line_top[x] && y <= line_bottom[x]) ? c_trace : y >= fill_from[x] ? c_fill : column_bg[x];
        }
        if (p->show_max) for (int x = 0; x < W; x++) if (max_y[x] == y && y < line_top[x]) row[x] = c_max;
        if (tune_x >= 0 && ((y >> 2) & 1)) row[tune_x] = c_tune;
        if (p->marker_x >= 0 && p->marker_x < W) row[p->marker_x] = c_marker;
    }
    esp_cache_msync(fb + area.y * DISPLAY_WIDTH, Hs * DISPLAY_WIDTH * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    prof[2] = esp_timer_get_time();
    /* Waterfall: ring rows [image_head, Hw) then [0, image_head). The caller
     * holds the LVGL lock, so LVGL is never mid-flush on these rows. */
    int top = area.y + Hs + area.gap;
    if (Hw) {
        image_copy(image_head, Hw - image_head, top);
        image_copy(0, image_head, top + Hw - image_head);
        if (p->marker_x >= 0 && p->marker_x < W) {
            for (int y = top + 1; y < top + Hw; y += 2) fb[y * DISPLAY_WIDTH + area.x + p->marker_x] = c_marker;
            esp_cache_msync(fb + top * DISPLAY_WIDTH, Hw * DISPLAY_WIDTH * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        }
    }
    prof[3] = esp_timer_get_time();
    snprintf(profile_text, sizeof(profile_text), "columns %lld us, spectrum %lld us, waterfall %lld us",
             (long long)(prof[1] - prof[0]), (long long)(prof[2] - prof[1]), (long long)(prof[3] - prof[2]));
}

bool plot_trace_percentiles(float *p20, float *peak) {
    if (!trace_valid || bins < 8) return false;
    unsigned hist[256] = {0};
    for (unsigned j = 0; j < bins; j++) hist[db_code(trace[j])]++;
    unsigned target = bins / 5, seen = 0, p20_code = 0, peak_code = 0;
    for (unsigned c = 0; c < 256; c++) {
        seen += hist[c];
        if (!p20_code && seen >= target) p20_code = c;
        if (hist[c]) peak_code = c;
    }
    *p20 = p20_code / 2.0f - 160.0f;
    *peak = peak_code / 2.0f - 160.0f;
    return true;
}

bool plot_trace_at(int x, float *db) {
    if (!trace_valid || !bins || x < 0 || x >= area.width) return false;
    ensure_map(bins, area.width);
    *db = column_value(trace, bins, area.width, x);
    return true;
}
