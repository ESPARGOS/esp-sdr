#pragma once
#include <stdbool.h>
#include <stdint.h>

#define PLOT_MAX_WIDTH 800
#define PLOT_MAX_WATERFALL 400
#define PLOT_MAX_BINS 2048

typedef struct { int x, y, width, spectrum_height, gap, waterfall_height; } plot_area_t;

typedef struct {
    float floor_db, range_db;
    float average;        /* trace IIR weight of the previous trace, 0 = off */
    bool show_max;        /* draw the max-hold trace */
    bool fill_dc;         /* replace the three centre bins with their neighbours */
    double center_hz;     /* LO frequency: centre of the displayed window */
    double span_hz;
    double tune_hz;       /* requested centre, dashed line when offset from center_hz */
    int marker_x;         /* plot column, -1 for none */
} plot_params_t;

typedef struct { double step_hz, minor_hz; } plot_ticks_t;
/* Axis tick spacing for a span over width pixels (same rule as ESP-WebSDR). */
plot_ticks_t plot_ticks(double span_hz, int width);

void plot_init(void);
void plot_set_area(const plot_area_t *area);   /* also clears */
void plot_clear(void);                         /* history, trace and maximum */
void plot_reset_traces(void);                  /* trace and maximum only */
void plot_push(const float *db, unsigned bins, const plot_params_t *p);
/* Draw spectrum and waterfall straight into the panel frame buffer. The
 * caller holds the LVGL lock. */
void plot_render(const plot_params_t *p);
bool plot_trace_percentiles(float *p20, float *peak);
/* Trace level at a plot column; false when no trace exists. */
bool plot_trace_at(int x, float *db);
/* Last frame's phase timings as text (diagnostics). */
const char *plot_profile(void);
