/* LVGL user interface, laid out for a 4.3" 800x480 touch panel.
 *
 * The plot (spectrum, frequency axis, waterfall) spans the full width. A
 * top bar carries the logo, the frequency readout (tap: keypad) and the
 * status lines. A bottom toolbar of six finger-sized buttons opens one of
 * four "sheets" (Tune, Span, RX, View) over the waterfall; the spectrum
 * stays live above the sheet while settings change. Every control is at
 * least 56 px tall. */
#include "ui.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "analyzer.h"
#include "display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "plot.h"
#include "radio.h"
#include "rx_bandwidth.h"
#include "rx_tuning.h"

static const char *TAG = "ui";
extern const lv_image_dsc_t espargos_logo;   /* logo.c, alpha-only wordmark */

/* LVGL's own formatter has no floating point support. */
static void label_printf(lv_obj_t *label, const char *fmt, ...) {
    char text[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    lv_label_set_text(label, text);
}

/* Vertical budget: 56 + 160 + 20 + 180 + 64 = 480. */
#define TOP_H 56
#define SPECTRUM_H 160
#define AXIS_H 20
#define WATERFALL_H 180
#define TOOLBAR_H 64
#define DB_STRIP_W 40
#define SHEET_Y (TOP_H + SPECTRUM_H + AXIS_H)
#define SHEET_H WATERFALL_H
#define TOOLBAR_Y (SHEET_Y + SHEET_H)
#define CONTROL_H 56
#define MAX_TICKS 16
#define COLOR_BG 0x11191e
#define COLOR_PANEL 0x182126
#define COLOR_LINE 0x293135
#define COLOR_TEXT 0xdee2e6
#define COLOR_MUTED 0xa1adb2
#define COLOR_ACCENT 0x37c964
#define COLOR_MARKER 0xffe3a0

typedef struct {
    uint32_t magic;
    unsigned frequency_mhz;
    bool agc; unsigned gain;
    bool bandwidth_open; unsigned bandwidth_mhz;
    unsigned rate_code, nfft_index;
    bool max_detector; unsigned dc_mode;      /* 0 raw, 1 fill, 2 offset LO */
    int floor_db, range_db;
    unsigned average_index;
    bool hold, autoscale, paused;
} settings_t;
#define SETTINGS_MAGIC 0x4b56554bu
static settings_t st = {SETTINGS_MAGIC, 2437, true, 40, false, 20, 0, 3, false, 0, -100, 80, 1, false, false, false};
static const unsigned nfft_options[] = {256, 512, 1024, 2048};
static const float average_options[] = {0, 0.3f, 0.65f, 0.9f};
enum { CH_RADIO = 1, CH_ANALYZER = 2, CH_CLEAR = 4, CH_TRACES = 8, CH_LAYOUT = 16 };
enum { SHEET_TUNE, SHEET_SPAN, SHEET_RX, SHEET_VIEW, SHEET_COUNT, TOOL_CLEAR = SHEET_COUNT, TOOL_PAUSE, TOOL_COUNT };

static lv_display_t *display;
static lv_obj_t *frequency_button, *frequency_label, *tune_frequency_label, *status_label;
static lv_obj_t *db_labels[5], *axis_strip, *axis_labels[MAX_TICKS], *plot_touch;
static lv_obj_t *tools[TOOL_COUNT], *pause_label, *sheet, *pages[SHEET_COUNT];
static lv_obj_t *gain_matrix, *gain_slider, *gain_label, *bandwidth_matrix, *bandwidth_slider, *bandwidth_label;
static lv_obj_t *rate_matrix, *nfft_matrix, *dc_matrix, *detector_matrix, *average_matrix;
static lv_obj_t *floor_slider, *floor_label, *range_slider, *range_label, *hold_button, *autoscale_button;
static lv_obj_t *modal, *keypad_text;
static lv_timer_t *save_timer;
static plot_area_t area;
static int marker_x = -1, open_sheet = -1;
static volatile bool modal_open;
static unsigned frames;
static analyzer_stats_t last_stats;
static float fps, render_ms, fetch_ms, period_ms, wake_ms;
static float auto_peak = NAN;

static unsigned lo_offset_mhz(void) {
    if (st.dc_mode != 2) return 0;
    return radio_rate_hz(st.rate_code) >= 40000000 ? 5 : 2;
}
static unsigned lo_frequency(void) {
    unsigned o = lo_offset_mhz();
    if (!o) return st.frequency_mhz;
    if (rx_frequency_valid(st.frequency_mhz - o)) return st.frequency_mhz - o;
    if (rx_frequency_valid(st.frequency_mhz + o)) return st.frequency_mhz + o;
    return st.frequency_mhz;
}
static plot_params_t params(void) {
    return (plot_params_t){
        .floor_db = st.floor_db, .range_db = st.range_db,
        .average = average_options[st.average_index], .show_max = st.hold,
        .fill_dc = st.dc_mode == 1,
        .center_hz = lo_frequency() * 1e6, .span_hz = radio_rate_hz(st.rate_code),
        .tune_hz = st.frequency_mhz * 1e6, .marker_x = marker_x,
    };
}
static void format_hz(char *out, size_t size, double hz) {
    double a = fabs(hz);
    if (a >= 1e6) snprintf(out, size, "%.3f MHz", hz / 1e6);
    else if (a >= 1e3) snprintf(out, size, "%.1f kHz", hz / 1e3);
    else snprintf(out, size, "%.0f Hz", hz);
}

/* ---- persistence ---------------------------------------------------- */
static void settings_load(void) {
    nvs_handle_t h;
    if (nvs_open("korvo", NVS_READONLY, &h) != ESP_OK) return;
    settings_t loaded; size_t size = sizeof(loaded);
    if (nvs_get_blob(h, "settings", &loaded, &size) == ESP_OK && size == sizeof(loaded) && loaded.magic == SETTINGS_MAGIC) {
        loaded.paused = false;
        if (rx_frequency_valid(loaded.frequency_mhz) && loaded.rate_code < RADIO_RATE_COUNT && loaded.nfft_index < 4 &&
            loaded.dc_mode < 3 && loaded.average_index < 4 && loaded.floor_db >= -140 && loaded.floor_db <= -40 &&
            loaded.range_db >= 20 && loaded.range_db <= 120)
            st = loaded;
    }
    nvs_close(h);
}
static void settings_save_cb(lv_timer_t *t) {
    lv_timer_pause(t);
    /* Flash writes stall the other core; never race the capture critical section. */
    analyzer_set_paused(true);
    analyzer_wait_idle();
    nvs_handle_t h;
    if (nvs_open("korvo", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "settings", &st, sizeof(st));
        nvs_commit(h);
        nvs_close(h);
    }
    analyzer_set_paused(st.paused);
}
static void schedule_save(void) { lv_timer_reset(save_timer); lv_timer_resume(save_timer); }

/* ---- labels ----------------------------------------------------------- */
/* Grey out a control and stop it from reacting to touch. */
static void enable(lv_obj_t *o, bool on) {
    if (on) lv_obj_remove_state(o, LV_STATE_DISABLED); else lv_obj_add_state(o, LV_STATE_DISABLED);
    lv_obj_set_clickable(o, on);
}
static void update_axis(void) {
    double span = radio_rate_hz(st.rate_code), f0 = lo_frequency() * 1e6 - span / 2;
    plot_ticks_t t = plot_ticks(span, area.width);
    int decimals = (int)fmax(0, ceil(-log10(t.step_hz / 1e6) - 1e-9));
    unsigned used = 0;
    for (double f = ceil(f0 / t.step_hz) * t.step_hz; f <= f0 + span && used < MAX_TICKS; f += t.step_hz) {
        double r = round(f / t.step_hz) * t.step_hz;
        int x = (int)lrint((r - f0) / span * area.width);
        lv_obj_t *label = axis_labels[used++];
        label_printf(label, "%.*f", decimals, r / 1e6);
        lv_obj_update_layout(label);
        int w = lv_obj_get_width(label), lx = x - w / 2;
        if (lx < 0) lx = 0;
        if (lx + w > area.width) lx = area.width - w;
        lv_obj_set_pos(label, lx, 2);
        lv_obj_set_hidden(label, false);
    }
    for (; used < MAX_TICKS; used++) lv_obj_set_hidden(axis_labels[used], true);
    for (int i = 0; i <= 4; i++) {
        label_printf(db_labels[i], "%d", (int)lrintf(st.floor_db + st.range_db - i * st.range_db / 4.0f));
        int y = area.y + (int)lrintf(i * (SPECTRUM_H - 1) / 4.0f) - (i == 0 ? 0 : i == 4 ? 14 : 7);
        lv_obj_set_pos(db_labels[i], 4, y);
    }
}
static void update_controls(void) {
    label_printf(frequency_label, "%u MHz", st.frequency_mhz);
    label_printf(tune_frequency_label, "%u MHz", st.frequency_mhz);
    lv_buttonmatrix_set_button_ctrl(gain_matrix, st.agc ? 0 : 1, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_slider_set_range(gain_slider, 0, radio_gain_max());
    lv_slider_set_value(gain_slider, st.gain, LV_ANIM_OFF);
    enable(gain_slider, !st.agc);
    label_printf(gain_label, st.agc ? "Gain: hardware AGC" : "Gain: manual index %u / %u", st.gain, radio_gain_max());
    lv_buttonmatrix_set_button_ctrl(bandwidth_matrix, st.bandwidth_open ? 0 : 1, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_slider_set_value(bandwidth_slider, st.bandwidth_mhz, LV_ANIM_OFF);
    enable(bandwidth_slider, !st.bandwidth_open);
    label_printf(bandwidth_label, st.bandwidth_open ? "Analog bandwidth: open (widest)" : "Analog bandwidth: filter %u MHz", st.bandwidth_mhz);
    lv_buttonmatrix_set_button_ctrl(rate_matrix, st.rate_code, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_buttonmatrix_set_button_ctrl(nfft_matrix, st.nfft_index, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_buttonmatrix_set_button_ctrl(dc_matrix, st.dc_mode, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_buttonmatrix_set_button_ctrl(detector_matrix, st.max_detector ? 1 : 0, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_buttonmatrix_set_button_ctrl(average_matrix, st.average_index, LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_slider_set_value(floor_slider, st.floor_db, LV_ANIM_OFF);
    label_printf(floor_label, "Floor %d dBFS", st.floor_db);
    lv_slider_set_value(range_slider, st.range_db, LV_ANIM_OFF);
    label_printf(range_label, "Range %d dB (%g dB/div)", st.range_db, st.range_db / 4.0);
    if (st.hold) lv_obj_add_state(hold_button, LV_STATE_CHECKED); else lv_obj_remove_state(hold_button, LV_STATE_CHECKED);
    if (st.autoscale) lv_obj_add_state(autoscale_button, LV_STATE_CHECKED); else lv_obj_remove_state(autoscale_button, LV_STATE_CHECKED);
    if (st.paused) lv_obj_add_state(tools[TOOL_PAUSE], LV_STATE_CHECKED); else lv_obj_remove_state(tools[TOOL_PAUSE], LV_STATE_CHECKED);
    lv_label_set_text(pause_label, st.paused ? LV_SYMBOL_PLAY " Resume" : LV_SYMBOL_PAUSE " Pause");
}
static void update_status(void) {
    double span = radio_rate_hz(st.rate_code);
    unsigned n = nfft_options[st.nfft_index];
    char rbw[24], third[96];
    format_hz(rbw, sizeof(rbw), 1.44 * span / n);
    float db;
    if (marker_x >= 0 && plot_trace_at(marker_x, &db)) {
        double f = lo_frequency() * 1e6 - span / 2 + (marker_x + 0.5) / area.width * span;
        char text[24]; format_hz(text, sizeof(text), f);
        snprintf(third, sizeof(third), "#%06x Marker %s, %.1f dBFS#", COLOR_MARKER, text, (double)db);
    } else snprintf(third, sizeof(third), "Touch the plot for a marker");
    label_printf(status_label, "%g MS/s, %u bins, RBW %s\n%s, %.0f fps, %.1fk FFT/s, %.0f%% RF\n%s",
                 span / 1e6, n, rbw, st.agc ? "AGC" : "Manual", (double)fps, last_stats.ffts_per_second / 1000.0,
                 last_stats.coverage * 100.0, third);
}

/* ---- apply ----------------------------------------------------------- */
static void layout(void) {
    bool sheet_open = open_sheet >= 0;
    area = (plot_area_t){.x = DB_STRIP_W, .y = TOP_H, .width = DISPLAY_WIDTH - DB_STRIP_W, .spectrum_height = SPECTRUM_H,
                         .gap = AXIS_H, .waterfall_height = sheet_open ? 0 : WATERFALL_H};
    plot_set_area(&area);
    lv_obj_set_pos(plot_touch, area.x, area.y);
    lv_obj_set_size(plot_touch, area.width, SPECTRUM_H + AXIS_H + area.waterfall_height);
    lv_obj_set_pos(axis_strip, area.x, area.y + SPECTRUM_H);
    lv_obj_set_size(axis_strip, area.width, AXIS_H);
    for (int i = 0; i < SHEET_COUNT; i++) {
        lv_obj_set_hidden(pages[i], i != open_sheet);
        if (i == open_sheet) lv_obj_add_state(tools[i], LV_STATE_CHECKED); else lv_obj_remove_state(tools[i], LV_STATE_CHECKED);
    }
    lv_obj_set_hidden(sheet, !sheet_open);
}
static void apply(unsigned what) {
    if (what & CH_LAYOUT) layout();
    if (what & CH_RADIO) {
        radio_settings_t r = {lo_frequency(), st.agc, st.gain, st.bandwidth_open ? 0 : st.bandwidth_mhz};
        if (!radio_apply(&r)) ESP_LOGW(TAG, "radio rejected settings");
    }
    if (what & CH_ANALYZER) {
        analyzer_config_t a = {st.rate_code, nfft_options[st.nfft_index], st.max_detector, st.dc_mode == 1};
        analyzer_set_config(&a);
    }
    if (what & CH_CLEAR) { marker_x = -1; plot_clear(); }
    else if (what & CH_TRACES) plot_reset_traces();
    analyzer_set_paused(st.paused);
    update_controls();
    update_axis();
    update_status();
    schedule_save();
}
static void set_frequency(unsigned mhz) {
    if (!rx_frequency_valid(mhz) || mhz == st.frequency_mhz) return;
    st.frequency_mhz = mhz;
    apply(CH_RADIO | CH_CLEAR);
}
static void show_sheet(int index) {
    open_sheet = index == open_sheet ? -1 : index;
    layout();
}

/* ---- events ----------------------------------------------------------- */
static unsigned selected(lv_event_t *e) { return lv_buttonmatrix_get_selected_button(lv_event_get_target_obj(e)); }
static void step_cb(lv_event_t *e) { set_frequency(st.frequency_mhz + (int)(intptr_t)lv_event_get_user_data(e)); }
static void gain_matrix_cb(lv_event_t *e) { st.agc = selected(e) == 0; apply(CH_RADIO | CH_TRACES); }
static void gain_slider_cb(lv_event_t *e) {
    st.gain = lv_slider_get_value(lv_event_get_target_obj(e));
    label_printf(gain_label, "Gain: manual index %u / %u", st.gain, radio_gain_max());
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) apply(CH_RADIO | CH_TRACES);
}
static void bandwidth_matrix_cb(lv_event_t *e) { st.bandwidth_open = selected(e) == 0; apply(CH_RADIO | CH_TRACES); }
static void bandwidth_slider_cb(lv_event_t *e) {
    st.bandwidth_mhz = lv_slider_get_value(lv_event_get_target_obj(e));
    label_printf(bandwidth_label, "Analog bandwidth: filter %u MHz", st.bandwidth_mhz);
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) apply(CH_RADIO | CH_TRACES);
}
static void rate_cb(lv_event_t *e) { st.rate_code = selected(e); apply(CH_RADIO | CH_ANALYZER | CH_CLEAR); }
static void nfft_cb(lv_event_t *e) { st.nfft_index = selected(e); apply(CH_ANALYZER | CH_CLEAR); }
static void dc_cb(lv_event_t *e) { st.dc_mode = selected(e); apply(CH_RADIO | CH_ANALYZER | CH_CLEAR); }
static void detector_cb(lv_event_t *e) { st.max_detector = selected(e) == 1; apply(CH_ANALYZER | CH_TRACES); }
static void average_cb(lv_event_t *e) { st.average_index = selected(e); apply(CH_TRACES); }
static void floor_cb(lv_event_t *e) {
    st.floor_db = lv_slider_get_value(lv_event_get_target_obj(e));
    label_printf(floor_label, "Floor %d dBFS", st.floor_db);
    update_axis();
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) schedule_save();
}
static void range_cb(lv_event_t *e) {
    st.range_db = lv_slider_get_value(lv_event_get_target_obj(e));
    label_printf(range_label, "Range %d dB (%g dB/div)", st.range_db, st.range_db / 4.0);
    update_axis();
    if (lv_event_get_code(e) == LV_EVENT_RELEASED) schedule_save();
}
static void hold_cb(lv_event_t *e) { st.hold = lv_obj_has_state(lv_event_get_target_obj(e), LV_STATE_CHECKED); apply(CH_TRACES); }
static void autoscale_cb(lv_event_t *e) {
    st.autoscale = lv_obj_has_state(lv_event_get_target_obj(e), LV_STATE_CHECKED);
    auto_peak = NAN;
    apply(0);
}
static void tool_cb(lv_event_t *e) {
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    if (i < SHEET_COUNT) show_sheet(i);
    else if (i == TOOL_CLEAR) apply(CH_CLEAR);
    else { st.paused = !st.paused; apply(0); }
}
static void status_cb(lv_event_t *e) { (void)e; marker_x = -1; update_status(); }
static void preset_cb(lv_event_t *e) {
    static const unsigned presets[] = {2412, 2437, 2462, 2402, 2426, 2480};
    unsigned i = selected(e);
    if (i < 6) set_frequency(presets[i]);
}
/* Touching the plot only ever moves the marker; tuning stays with the Tune
 * sheet and the keypad, so a slow drag can never retune by accident. */
static void plot_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
        lv_point_t p; lv_indev_get_point(indev, &p);
        int x = p.x - area.x;
        marker_x = x < 0 ? 0 : x >= area.width ? area.width - 1 : x;
        if (code == LV_EVENT_PRESSED) update_status();
    } else if (code == LV_EVENT_RELEASED) update_status();
}

/* ---- frequency keypad ------------------------------------------------- */
/* The modal is built once and shown/hidden: deleting widgets from inside
 * their own event handlers is what LVGL warns against. While it is shown the
 * render task leaves the frame buffer alone. */
static void keypad_close(void) { lv_obj_set_hidden(modal, true); modal_open = false; }
static void keypad_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        unsigned mhz = (unsigned)lrint(atof(lv_textarea_get_text(keypad_text)));
        if (rx_frequency_valid(mhz)) { keypad_close(); set_frequency(mhz); return; }
        lv_textarea_set_text(keypad_text, "");
    } else if (code == LV_EVENT_CANCEL) keypad_close();
}
static void keypad_cancel_cb(lv_event_t *e) { (void)e; keypad_close(); }
static void keypad_open(lv_event_t *e) {
    (void)e;
    if (modal_open) return;
    lv_textarea_set_text(keypad_text, "");
    modal_open = true;
    lv_obj_set_hidden(modal, false);
}

/* ---- widgets ----------------------------------------------------------- */
static lv_obj_t *section(lv_obj_t *parent, const char *text) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    return l;
}
/* Transparent flex container. */
static lv_obj_t *box(lv_obj_t *parent, lv_flex_flow_t flow, int w, int h) {
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_radius(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_pad_gap(o, 8, 0);
    lv_obj_set_scrollable(o, false);
    lv_obj_set_flex_flow(o, flow);
    lv_obj_set_flex_align(o, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    return o;
}
/* A titled column inside a sheet line; *label receives the title label. */
static lv_obj_t *group(lv_obj_t *line, const char *title, int w, lv_obj_t **label) {
    lv_obj_t *g = box(line, LV_FLEX_FLOW_COLUMN, w, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_gap(g, 4, 0);
    lv_obj_set_flex_align(g, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_t *l = section(g, title);
    if (label) *label = l;
    return g;
}
static lv_obj_t *matrix(lv_obj_t *parent, const char *const map[], int w, lv_event_cb_t cb) {
    lv_obj_t *m = lv_buttonmatrix_create(parent);
    lv_buttonmatrix_set_map(m, map);
    lv_buttonmatrix_set_button_ctrl_all(m, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(m, true);
    lv_obj_set_size(m, w, CONTROL_H);
    lv_obj_set_style_pad_all(m, 0, 0);
    lv_obj_set_style_pad_gap(m, 6, 0);
    lv_obj_set_style_bg_opa(m, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(m, 0, 0);
    lv_obj_add_event_cb(m, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return m;
}
static lv_obj_t *slider(lv_obj_t *parent, int min, int max, lv_event_cb_t cb) {
    lv_obj_t *s = lv_slider_create(parent);
    lv_slider_set_range(s, min, max);
    lv_obj_set_height(s, 14);
    lv_obj_set_flex_grow(s, 1);
    lv_obj_set_style_margin_hor(s, 18, 0);
    lv_obj_set_style_pad_all(s, 11, LV_PART_KNOB);   /* 36 px knob */
    lv_obj_set_ext_click_area(s, 20);
    lv_obj_set_style_opa(s, LV_OPA_40, LV_STATE_DISABLED);
    lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s, cb, LV_EVENT_RELEASED, NULL);
    return s;
}
static lv_obj_t *button(lv_obj_t *parent, const char *text, int w, int h, const lv_font_t *font, lv_event_cb_t cb, void *data) {
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, text);
    if (font) lv_obj_set_style_text_font(l, font, 0);
    lv_obj_center(l);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, data);
    return b;
}
static lv_obj_t *toggle(lv_obj_t *parent, const char *text, int w, lv_event_cb_t cb) {
    lv_obj_t *b = button(parent, text, w, CONTROL_H, NULL, NULL, NULL);
    lv_obj_set_checkable(b, true);
    lv_obj_add_event_cb(b, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return b;
}
static lv_obj_t *page(const char *unused) {
    (void)unused;
    lv_obj_t *p = box(sheet, LV_FLEX_FLOW_COLUMN, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_gap(p, 8, 0);
    return p;
}
static lv_obj_t *line(lv_obj_t *p) { return box(p, LV_FLEX_FLOW_ROW, LV_PCT(100), LV_SIZE_CONTENT); }

static void build_keypad(lv_obj_t *screen) {
    modal = lv_obj_create(screen);
    lv_obj_set_size(modal, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_set_pos(modal, 0, 0);
    lv_obj_set_style_bg_color(modal, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(modal, 0, 0);
    lv_obj_set_style_radius(modal, 0, 0);
    lv_obj_set_scrollable(modal, false);
    lv_obj_set_hidden(modal, true);
    lv_obj_t *panel = lv_obj_create(modal);
    lv_obj_set_size(panel, 500, 400);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(COLOR_LINE), 0);
    lv_obj_set_style_pad_all(panel, 12, 0);
    lv_obj_set_scrollable(panel, false);
    lv_obj_t *title = lv_label_create(panel);
    label_printf(title, "Center frequency in MHz (%u to %u)", RX_FREQ_MIN, RX_FREQ_MAX);
    lv_obj_set_style_text_color(title, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 20);
    lv_obj_t *cancel = button(panel, LV_SYMBOL_CLOSE " Cancel", 130, CONTROL_H, NULL, keypad_cancel_cb, NULL);
    lv_obj_align(cancel, LV_ALIGN_TOP_RIGHT, 0, 0);
    keypad_text = lv_textarea_create(panel);
    lv_textarea_set_one_line(keypad_text, true);
    lv_textarea_set_accepted_chars(keypad_text, "0123456789");
    lv_textarea_set_max_length(keypad_text, 4);
    lv_textarea_set_placeholder_text(keypad_text, "2437");
    lv_obj_set_style_text_font(keypad_text, &lv_font_montserrat_28, 0);
    lv_obj_set_size(keypad_text, 474, 56);
    lv_obj_align(keypad_text, LV_ALIGN_TOP_LEFT, 0, 64);
    lv_obj_t *kb = lv_keyboard_create(panel);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb, keypad_text);
    lv_obj_set_size(kb, 474, 244);
    lv_obj_set_style_text_font(kb, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(kb, keypad_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(kb, keypad_cb, LV_EVENT_CANCEL, NULL);
}

static void build(void) {
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_scrollable(screen, false);

    /* Top bar: logo, frequency, status. */
    lv_obj_t *logo = lv_image_create(screen);
    lv_image_set_src(logo, &espargos_logo);
    lv_obj_set_style_image_recolor(logo, lv_color_hex(COLOR_ACCENT), 0);
    lv_obj_set_style_image_recolor_opa(logo, LV_OPA_COVER, 0);
    lv_obj_set_pos(logo, 16, (TOP_H - espargos_logo.header.h) / 2);
    frequency_button = lv_button_create(screen);
    lv_obj_set_size(frequency_button, 150, 48);
    lv_obj_set_pos(frequency_button, (DISPLAY_WIDTH - 150) / 2, 4);
    lv_obj_set_style_bg_color(frequency_button, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_color(frequency_button, lv_color_hex(COLOR_LINE), 0);
    lv_obj_set_style_border_width(frequency_button, 1, 0);
    frequency_label = lv_label_create(frequency_button);
    lv_obj_set_style_text_font(frequency_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(frequency_label, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_center(frequency_label);
    lv_obj_add_event_cb(frequency_button, keypad_open, LV_EVENT_CLICKED, NULL);
    status_label = lv_label_create(screen);
    lv_label_set_recolor(status_label, true);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_width(status_label, 300);
    lv_obj_set_pos(status_label, DISPLAY_WIDTH - 300 - 10, 3);
    lv_obj_set_clickable(status_label, true);
    lv_obj_add_event_cb(status_label, status_cb, LV_EVENT_CLICKED, NULL);

    /* Axis strips and the touch surface over the plot. */
    for (int i = 0; i <= 4; i++) {
        db_labels[i] = lv_label_create(screen);
        lv_obj_set_style_text_font(db_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(db_labels[i], lv_color_hex(COLOR_MUTED), 0);
    }
    axis_strip = lv_obj_create(screen);
    lv_obj_set_style_bg_color(axis_strip, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_width(axis_strip, 0, 0);
    lv_obj_set_style_radius(axis_strip, 0, 0);
    lv_obj_set_style_pad_all(axis_strip, 0, 0);
    lv_obj_set_scrollable(axis_strip, false);
    for (int i = 0; i < MAX_TICKS; i++) {
        axis_labels[i] = lv_label_create(axis_strip);
        lv_obj_set_style_text_font(axis_labels[i], &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(axis_labels[i], lv_color_hex(COLOR_MUTED), 0);
        lv_obj_set_hidden(axis_labels[i], true);
    }
    plot_touch = lv_obj_create(screen);
    lv_obj_set_style_bg_opa(plot_touch, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(plot_touch, 0, 0);
    lv_obj_set_style_radius(plot_touch, 0, 0);
    lv_obj_set_style_pad_all(plot_touch, 0, 0);
    lv_obj_set_scrollable(plot_touch, false);
    lv_obj_set_clickable(plot_touch, true);
    lv_obj_add_event_cb(plot_touch, plot_cb, LV_EVENT_ALL, NULL);

    /* Sheets over the waterfall. */
    sheet = lv_obj_create(screen);
    lv_obj_set_pos(sheet, 0, SHEET_Y);
    lv_obj_set_size(sheet, DISPLAY_WIDTH, SHEET_H);
    lv_obj_set_style_bg_color(sheet, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_color(sheet, lv_color_hex(COLOR_LINE), 0);
    lv_obj_set_style_border_side(sheet, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_radius(sheet, 0, 0);
    lv_obj_set_style_pad_hor(sheet, 10, 0);
    lv_obj_set_style_pad_ver(sheet, 8, 0);
    lv_obj_set_scrollable(sheet, false);
    lv_obj_set_hidden(sheet, true);
    const int half = (DISPLAY_WIDTH - 20 - 8) / 2;

    lv_obj_t *p = pages[SHEET_TUNE] = page("Tune");
    lv_obj_t *g = group(line(p), "Center frequency", LV_PCT(100), NULL);
    lv_obj_t *row = line(g);
    static const struct { const char *text; int step; } steps[] = {{"-10", -10}, {"-5", -5}, {"-1", -1}, {"+1", 1}, {"+5", 5}, {"+10", 10}};
    for (int i = 0; i < 6; i++) {
        if (i == 3) {
            lv_obj_t *b = button(row, "", 220, CONTROL_H, &lv_font_montserrat_28, keypad_open, NULL);
            lv_obj_set_style_bg_color(b, lv_color_hex(COLOR_BG), 0);
            tune_frequency_label = lv_obj_get_child(b, 0);
        }
        button(row, steps[i].text, 84, CONTROL_H, &lv_font_montserrat_20, step_cb, (void *)(intptr_t)steps[i].step);
    }
    g = group(line(p), "Presets", LV_PCT(100), NULL);
    static const char *preset_map[] = {"Wi-Fi 1", "Wi-Fi 6", "Wi-Fi 11", "BLE 37", "BLE 38", "BLE 39", ""};
    lv_obj_t *presets = matrix(g, preset_map, LV_PCT(100), preset_cb);
    lv_buttonmatrix_clear_button_ctrl_all(presets, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(presets, false);

    p = pages[SHEET_SPAN] = page("Span");
    g = group(line(p), "Sample rate = displayed span", LV_PCT(100), NULL);
    static const char *rate_map[] = {"80 MS/s", "40 MS/s", "20 MS/s", "10 MS/s", "8 MS/s", "4 MS/s", ""};
    rate_matrix = matrix(g, rate_map, LV_PCT(100), rate_cb);
    row = line(p);
    g = group(row, "FFT bins", half, NULL);
    static const char *nfft_map[] = {"256", "512", "1024", "2048", ""};
    nfft_matrix = matrix(g, nfft_map, LV_PCT(100), nfft_cb);
    g = group(row, "0 Hz (zero-IF): the LO leaves a spike at the centre", half, NULL);
    static const char *dc_map[] = {"Show as is", "Fill DC bins", "Offset LO", ""};
    dc_matrix = matrix(g, dc_map, LV_PCT(100), dc_cb);

    p = pages[SHEET_RX] = page("RX");
    g = group(line(p), "", LV_PCT(100), &gain_label);
    row = line(g);
    static const char *gain_map[] = {"AGC", "Manual", ""};
    gain_matrix = matrix(row, gain_map, 220, gain_matrix_cb);
    gain_slider = slider(row, 0, 1, gain_slider_cb);
    g = group(line(p), "", LV_PCT(100), &bandwidth_label);
    row = line(g);
    static const char *bandwidth_map[] = {"Open", "Filter", ""};
    bandwidth_matrix = matrix(row, bandwidth_map, 220, bandwidth_matrix_cb);
    bandwidth_slider = slider(row, RX_BANDWIDTH_MIN, RX_BANDWIDTH_MAX, bandwidth_slider_cb);

    p = pages[SHEET_VIEW] = page("View");
    row = line(p);
    g = group(row, "", half, &floor_label);
    lv_obj_t *r2 = line(g); lv_obj_set_height(r2, CONTROL_H);
    floor_slider = slider(r2, -140, -40, floor_cb);
    g = group(row, "", half, &range_label);
    r2 = line(g); lv_obj_set_height(r2, CONTROL_H);
    range_slider = slider(r2, 20, 120, range_cb);
    row = line(p);
    g = group(row, "Trace averaging", 330, NULL);
    static const char *average_map[] = {"Off", "Short", "Medium", "High", ""};
    average_matrix = matrix(g, average_map, LV_PCT(100), average_cb);
    g = group(row, "Detector", 190, NULL);
    static const char *detector_map[] = {"Average", "Peak", ""};
    detector_matrix = matrix(g, detector_map, LV_PCT(100), detector_cb);
    g = group(row, "Traces", 244, NULL);
    r2 = line(g);
    hold_button = toggle(r2, "Max hold", 118, hold_cb);
    autoscale_button = toggle(r2, "Auto scale", 118, autoscale_cb);

    /* Toolbar. */
    lv_obj_t *bar = box(screen, LV_FLEX_FLOW_ROW, DISPLAY_WIDTH - 12, TOOLBAR_H - 8);
    lv_obj_set_pos(bar, 6, TOOLBAR_Y + 4);
    lv_obj_set_style_pad_gap(bar, 6, 0);
    static const char *tool_text[TOOL_COUNT] = {"Tune", "Span", "RX", "View", LV_SYMBOL_TRASH " Clear", ""};
    for (int i = 0; i < TOOL_COUNT; i++) {
        tools[i] = button(bar, tool_text[i], 10, TOOLBAR_H - 8, &lv_font_montserrat_20, tool_cb, (void *)(intptr_t)i);
        lv_obj_set_flex_grow(tools[i], 1);
        lv_obj_set_style_bg_color(tools[i], lv_color_hex(COLOR_PANEL), 0);
        lv_obj_set_style_bg_color(tools[i], lv_color_hex(COLOR_ACCENT), LV_STATE_CHECKED);
        lv_obj_set_style_text_color(tools[i], lv_color_hex(0x0b1a10), LV_STATE_CHECKED);
    }
    pause_label = lv_obj_get_child(tools[TOOL_PAUSE], 0);

    build_keypad(screen);
}

/* ---- tasks ------------------------------------------------------------- */
static void render_task(void *arg) {
    (void)arg;
    float *db = heap_caps_malloc(ANALYZER_NFFT_MAX * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(db);
    unsigned n; analyzer_stats_t stats;
    int64_t last = 0;
    for (;;) {
        if (st.paused || modal_open) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        /* Pace to the panel refresh and always leave the UI core some idle time. */
        int64_t now = esp_timer_get_time();
        int64_t wait = 28000 - (now - last) - (int64_t)(fetch_ms * 1000);   /* the fetch wait is part of the period */
        vTaskDelay(wait > 1000 && wait < 28000 ? pdMS_TO_TICKS((wait + 999) / 1000) : 1);
        int64_t before_fetch = esp_timer_get_time();
        if (!analyzer_fetch(db, &n, &stats, pdMS_TO_TICKS(250))) continue;
        int64_t previous = last;
        last = esp_timer_get_time();
        fetch_ms = fetch_ms * 0.9f + (last - before_fetch) * 1e-3f * 0.1f;
        wake_ms = wake_ms * 0.9f + (last - stats.published_at) * 1e-3f * 0.1f;
        if (previous) period_ms = period_ms * 0.9f + (last - previous) * 1e-3f * 0.1f;
        /* The frame is drawn under the LVGL lock: widgets never change or
         * flush while the plot is written, and event handlers never wait
         * for the renderer. */
        lvgl_port_lock(0);
        if (!modal_open) {
            plot_params_t p = params();
            plot_push(db, n, &p);
            plot_render(&p);
            frames++;
        }
        lvgl_port_unlock();
        render_ms = render_ms * 0.9f + (esp_timer_get_time() - last) * 1e-3f * 0.1f;
        last_stats = stats;
    }
}
static void status_timer_cb(lv_timer_t *t) {
    (void)t;
    static int64_t last_time; static unsigned last_frames; static int64_t last_auto;
    int64_t now = esp_timer_get_time();
    if (last_time) fps = (frames - last_frames) * 1e6f / (now - last_time);
    last_time = now; last_frames = frames;
    update_status();
    if (st.autoscale && now - last_auto > 1000000 && !st.paused) {
        last_auto = now;
        float p20, peak;
        if (plot_trace_percentiles(&p20, &peak)) {
            float nf = p20 - 10;
            auto_peak = isnan(auto_peak) ? peak : fmaxf(peak, auto_peak - 2);
            int floor = fabsf(nf - st.floor_db) > 4 ? (int)lrintf(nf / 5) * 5 : st.floor_db;
            int top = st.floor_db + st.range_db;
            if (auto_peak + 5 > top || auto_peak + 25 < top) top = (int)ceilf((auto_peak + 5) / 10) * 10;
            floor = floor < -140 ? -140 : floor > -40 ? -40 : floor;
            int range = top - floor; range = range < 20 ? 20 : range > 120 ? 120 : range;
            if (floor != st.floor_db || range != st.range_db) {
                st.floor_db = floor; st.range_db = range;
                update_controls(); update_axis(); schedule_save();
            }
        }
    }
}

/* Console commands for scripted screenshots and remote checks. */
bool ui_command(const char *line) {
    unsigned value;
    char extra;
    bool known = true;
    lvgl_port_lock(0);
    if (sscanf(line, "SHEET %u %c", &value, &extra) == 1 && value <= SHEET_COUNT) { open_sheet = value < SHEET_COUNT ? (int)value : -1; layout(); }
    else if (sscanf(line, "FREQ %u %c", &value, &extra) == 1) set_frequency(value);
    else if (sscanf(line, "RATE %u %c", &value, &extra) == 1 && value < RADIO_RATE_COUNT) { st.rate_code = value; apply(CH_RADIO | CH_ANALYZER | CH_CLEAR); }
    else if (sscanf(line, "NFFT %u %c", &value, &extra) == 1 && value < 4) { st.nfft_index = value; apply(CH_ANALYZER | CH_CLEAR); }
    else if (sscanf(line, "DC %u %c", &value, &extra) == 1 && value < 3) { st.dc_mode = value; apply(CH_RADIO | CH_ANALYZER | CH_CLEAR); }
    else if (sscanf(line, "GAIN %u %c", &value, &extra) == 1) { st.agc = false; st.gain = value; apply(CH_RADIO | CH_TRACES); }
    else if (!strcmp(line, "AGC")) { st.agc = true; apply(CH_RADIO | CH_TRACES); }
    else if (sscanf(line, "BW %u %c", &value, &extra) == 1) { st.bandwidth_open = !value; if (value) st.bandwidth_mhz = value; apply(CH_RADIO | CH_TRACES); }
    else if (sscanf(line, "MARKER %u %c", &value, &extra) == 1) { marker_x = (int)value; update_status(); }
    else if (!strcmp(line, "HOLD")) { st.hold = !st.hold; apply(CH_TRACES); }
    else if (!strcmp(line, "MAX")) { st.max_detector = !st.max_detector; apply(CH_ANALYZER | CH_TRACES); }
    else if (!strcmp(line, "PAUSE")) { st.paused = !st.paused; apply(0); }
    else if (!strcmp(line, "CLEAR")) apply(CH_CLEAR);
    else if (!strcmp(line, "KEYPAD")) { if (modal_open) keypad_close(); else keypad_open(NULL); }
    else if (!strcmp(line, "AUTOSCALE")) { st.autoscale = !st.autoscale; auto_peak = NAN; apply(0); }
    else known = false;
    lvgl_port_unlock();
    return known;
}

static void stats_log_cb(lv_timer_t *t) {
    (void)t;
    ESP_LOGI(TAG, "%.0f fps (%.1f ms period, %.1f ms fetch wait of which %.1f ms wake-up, %.1f ms render: %s), %.0f FFT/s, %.0f snapshots/s (%.0f us capture + %.0f us: %s), RF coverage %.1f%%, core 1 %.0f%%, failures %u, heap %u KiB",
             (double)fps, (double)period_ms, (double)fetch_ms, (double)wake_ms, (double)render_ms, plot_profile(), (double)last_stats.ffts_per_second, (double)last_stats.snapshots_per_second,
             (double)last_stats.capture_us, (double)last_stats.process_us, last_stats.profile,
             last_stats.coverage * 100.0, last_stats.load * 100.0, last_stats.failures,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
}

void ui_init(lv_display_t *disp) {
    display = disp;
    settings_load();
    lvgl_port_lock(0);
    lv_theme_t *theme = lv_theme_default_init(display, lv_color_hex(COLOR_ACCENT), lv_color_hex(0xe6b969), true, &lv_font_montserrat_16);
    lv_display_set_theme(display, theme);
    build();
    save_timer = lv_timer_create(settings_save_cb, 2000, NULL);
    lv_timer_pause(save_timer);
    lv_timer_create(status_timer_cb, 500, NULL);
    lv_timer_create(stats_log_cb, 5000, NULL);
    apply(CH_LAYOUT | CH_RADIO | CH_ANALYZER | CH_CLEAR);
    lv_timer_pause(save_timer);
    lvgl_port_unlock();
    xTaskCreatePinnedToCore(render_task, "render", 6144, NULL, 6, NULL, 0);
    ESP_LOGI(TAG, "ready");
}
