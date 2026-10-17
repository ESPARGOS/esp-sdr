#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#define ANALYZER_NFFT_MAX 2048u

typedef struct {
    unsigned rate_code;   /* radio rate code 0..5 */
    unsigned nfft;        /* 256..2048, power of two */
    bool max_hold;        /* per-bin maximum instead of mean power */
    bool dc_notch;        /* slow DC tracker removes the 0 Hz spike */
} analyzer_config_t;

typedef struct {
    unsigned ffts;            /* FFTs merged into this spectrum */
    unsigned snapshots;       /* snapshots merged into this spectrum */
    float ffts_per_second;
    float snapshots_per_second;
    float coverage;           /* fraction of RF time covered by FFTs, 0..1 */
    float load;               /* core-1 busy fraction, 0..1 */
    unsigned failures;        /* cumulative capture failures */
    float capture_us;         /* mean radio_snapshot() time per snapshot */
    float process_us;         /* mean FFT/accumulate time per snapshot */
    char profile[128];         /* per-block phase timings (diagnostics) */
    int64_t published_at;     /* esp_timer time of the publish (wake-up latency diagnostics) */
} analyzer_stats_t;

/* Core-1 worker: snapshot, window, SIMD FFT and power accumulation. */
void analyzer_start(void);
void analyzer_set_config(const analyzer_config_t *config);
void analyzer_set_paused(bool paused);
/* Block until the worker is idle (used around flash writes). */
void analyzer_wait_idle(void);
/* Wait for the next merged spectrum: nfft dBFS values from -fs/2 to +fs/2.
 * Accumulation restarts after each successful fetch. */
bool analyzer_fetch(float *db, unsigned *nfft, analyzer_stats_t *stats, TickType_t timeout);
