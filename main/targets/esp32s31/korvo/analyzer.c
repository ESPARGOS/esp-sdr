/* Spectrum engine for the Korvo LCD: a core-1 task captures TCM snapshots
 * back-to-back and reduces them with the S31 SIMD unpack/FFT kernels. The UI
 * core fetches the merged spectrum once per display frame. */
#include "analyzer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <string.h>
#include "dsps_fft2r.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "radio.h"
#include "spectrum_dc.h"

static const char *TAG = "analyzer";
unsigned spectrum_dc_mode = 1;   /* spectrum_dc.h: slow tracker */
extern int s31_fft2r_sc16_rnd(int16_t *, int, int16_t *);
extern void s31_unpack_iq10_win(const uint32_t *, int16_t *, const int16_t *, unsigned);
extern void s31_power_acc(const int16_t *, uint64_t *, unsigned);

/* SIMD kernels load from their stack and operands; keep everything in HP SRAM. */
static StaticTask_t task_tcb;
static StackType_t task_stack[12288 / sizeof(StackType_t)] __attribute__((aligned(16)));
static int16_t twiddles[ANALYZER_NFFT_MAX] __attribute__((aligned(16)));
static int16_t fft_data[2 * ANALYZER_NFFT_MAX] __attribute__((aligned(16)));
static int16_t window2[2 * ANALYZER_NFFT_MAX] __attribute__((aligned(16)));
static uint16_t *natural_index;   /* PSRAM: used once per published spectrum */
/* Mean mode sums |X|^2 in the SIMD accumulator lanes (64-bit, no scaling);
 * max-hold keeps a float per bin. The two never coexist. */
static union { uint64_t sum[ANALYZER_NFFT_MAX]; float max[ANALYZER_NFFT_MAX]; } power __attribute__((aligned(16)));
static float *published;   /* PSRAM: read once per display frame */
static float full_scale_power = 1.0f;

static SemaphoreHandle_t ready;
static TaskHandle_t worker;
static analyzer_config_t pending = {.rate_code = 0, .nfft = 2048, .max_hold = false, .dc_notch = true};
static _Atomic unsigned config_generation = 1;
static _Atomic bool paused, idle, fetch_request;
static _Atomic int64_t fetch_requested_at;
static unsigned limiter_total_us, respond_total_us, responses;
static analyzer_stats_t published_stats;
static unsigned published_nfft;

static inline float fast_log2(float x) {
    union { float f; uint32_t u; } v = {.f = x};
    if (v.u < 0x00800000u) return -126.0f;
    int exponent = (int)((v.u >> 23) & 255u) - 127;
    v.u = (v.u & 0x7fffffu) | 0x3f800000u;
    float m = v.f;
    float p = -1.7417939f + m * (2.8212026f + m * (-1.4699568f + m * (0.44717955f - 0.056570851f * m)));
    return (float)exponent + p;
}
static inline float to_db(float ratio) { return 3.0102999f * fast_log2(ratio); }
/* RV32 has no 64-bit integer to float instruction; libgcc's routine costs
 * more than the rest of the publish loop together. */
static inline float u64_to_float(uint64_t v) { return (float)(uint32_t)(v >> 32) * 4294967296.0f + (float)(uint32_t)v; }

static void setup(const analyzer_config_t *c) {
    unsigned n = c->nfft, logn = 0;
    while ((1u << logn) < n) logn++;
    for (unsigned j = 0; j < n; j++) {
        int16_t w = (int16_t)lrintf(16383.5f * (1 - cosf(2 * (float)M_PI * j / n)));
        window2[2*j] = window2[2*j+1] = w;
        unsigned b = 0;
        for (unsigned k = 0; k < logn; k++) b = b * 2 + ((j >> k) & 1);
        natural_index[j] = b;
    }
    memset(&power, 0, sizeof(power));
    /* Reference: a full-scale IQ10 tone through the same window and FFT. */
    /* SIMD loads cannot reach RTC RAM: DMA-capable memory is HP SRAM only. */
    uint32_t *tone = heap_caps_aligned_alloc(16, (n + 8) * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (tone) {
        for (unsigned j = 0; j < n + 8; j++) {
            float phase = 2 * (float)M_PI * (j % n) / 8.0f;
            int i = (int)lrintf(511.0f * cosf(phase)), q = (int)lrintf(511.0f * sinf(phase));
            tone[j] = ((uint32_t)i & 0x3ffu) | (((uint32_t)q & 0x3ffu) << 10);
        }
        s31_unpack_iq10_win(tone, fft_data, window2, n / 8);
        s31_fft2r_sc16_rnd(fft_data, (int)n, dsps_fft_w_table_sc16);
        float peak = 1.0f;
        for (unsigned j = 0; j < n; j++) {
            float re = fft_data[2*j], im = fft_data[2*j+1], p = re*re + im*im;
            if (p > peak) peak = p;
        }
        full_scale_power = peak;
        /* The SIMD accumulator must agree with the plain sum, with the SAR
         * register in whatever state the FFT leaves it. */
        memset(&power, 0, n * sizeof(uint64_t));
        s31_power_acc(fft_data, power.sum, n / 8);
        s31_power_acc(fft_data, power.sum, n / 8);
        unsigned mismatches = 0;
        for (unsigned j = 0; j < n; j++) {
            int32_t re = fft_data[2*j], im = fft_data[2*j+1];
            if (power.sum[j] != 2 * ((uint64_t)(uint32_t)(re*re) + (uint32_t)(im*im))) mismatches++;
        }
        if (mismatches) ESP_LOGE(TAG, "SIMD power accumulator mismatch in %u of %u bins", mismatches, n);
        memset(&power, 0, n * sizeof(uint64_t));
        heap_caps_free(tone);
        ESP_LOGD(TAG, "n=%u full-scale bin power %.0f", n, (double)peak);
    }
}

/* Max-hold detector: integer squaring with one conversion per bin. */
static void __attribute__((noinline, optimize("O3"))) accumulate_max(const int16_t *restrict x, float *restrict max, unsigned n) {
    #pragma GCC unroll 4
    for (unsigned j = 0; j < n; j++) {
        int32_t re = x[2*j], im = x[2*j+1];
        float p = (float)((uint32_t)(re*re) + (uint32_t)(im*im));
        if (p > max[j]) max[j] = p;
    }
}

static unsigned capture_total_us, process_total_us;
static uint32_t prof_unpack, prof_fft, prof_acc, prof_blocks;
static void publish(const analyzer_config_t *c, unsigned ffts, unsigned snapshots, int64_t interval_us,
                    unsigned busy_us, uint64_t samples) {
    unsigned n = c->nfft;
    int64_t publish_start = esp_timer_get_time();
    float scale = 1.0f / (full_scale_power * (c->max_hold || !ffts ? 1.0f : (float)ffts));
    for (unsigned j = 0; j < n; j++) {
        unsigned display = (natural_index[j] + n / 2) & (n - 1);
        float p = (c->max_hold ? power.max[j] : u64_to_float(power.sum[j])) * scale;
        published[display] = p > 1e-20f ? to_db(p) : -200.0f;
    }
    memset(&power, 0, n * sizeof(uint64_t));
    published_nfft = n;
    published_stats.published_at = esp_timer_get_time();
    float seconds = interval_us > 0 ? interval_us * 1e-6f : 1e-6f;
    published_stats.ffts = ffts;
    published_stats.snapshots = snapshots;
    published_stats.ffts_per_second = ffts / seconds;
    published_stats.snapshots_per_second = snapshots / seconds;
    published_stats.coverage = (float)((double)samples / seconds / radio_rate_hz(c->rate_code));
    published_stats.load = busy_us / (seconds * 1e6f);
    published_stats.failures = radio_snapshot_failures();
    published_stats.capture_us = snapshots ? (float)capture_total_us / snapshots : 0;
    published_stats.process_us = snapshots ? (float)process_total_us / snapshots : 0;
    capture_total_us = process_total_us = 0;
    if (prof_blocks) snprintf(published_stats.profile, sizeof(published_stats.profile), "per block unpack %u + fft %u + acc %u cycles; respond %u us, limiter %u us, publish %u us",
                              (unsigned)(prof_unpack / prof_blocks), (unsigned)(prof_fft / prof_blocks), (unsigned)(prof_acc / prof_blocks),
                              responses ? respond_total_us / responses : 0, limiter_total_us, (unsigned)(esp_timer_get_time() - publish_start));
    prof_unpack = prof_fft = prof_acc = prof_blocks = 0;
    limiter_total_us = respond_total_us = responses = 0;
}

static void worker_task(void *arg) {
    (void)arg;
    analyzer_config_t c = pending;
    unsigned generation = 0;
    unsigned ffts = 0, snapshots = 0, busy_us = 0;
    uint64_t samples = 0;
    int64_t interval_start = esp_timer_get_time();
    spectrum_dc_t dc = {0};
    for (;;) {
        if (atomic_load(&paused)) {
            atomic_store(&idle, true);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        atomic_store(&idle, false);
        unsigned wanted = atomic_load(&config_generation);
        if (wanted != generation) {
            generation = wanted;
            c = pending;
            setup(&c);
            ffts = snapshots = busy_us = 0; samples = 0;
            memset(&dc, 0, sizeof(dc));
            interval_start = esp_timer_get_time();
        }
        int64_t t0 = esp_timer_get_time();
        const uint32_t *words; unsigned elapsed;
        if (!radio_snapshot(RADIO_SNAPSHOT_SAMPLES, c.rate_code, &words, &elapsed)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int64_t t_captured = esp_timer_get_time();
        unsigned n = c.nfft, blocks = RADIO_SNAPSHOT_SAMPLES / n;
        for (unsigned b = 0; b < blocks; b++) {
            uint32_t c0 = esp_cpu_get_cycle_count();
            s31_unpack_iq10_win(words + b * n, fft_data, window2, n / 8);
            uint32_t c1 = esp_cpu_get_cycle_count();
            s31_fft2r_sc16_rnd(fft_data, (int)n, dsps_fft_w_table_sc16);
            uint32_t c2 = esp_cpu_get_cycle_count();
            if (c.dc_notch) spectrum_dc_apply(&dc, fft_data, n);
            if (c.max_hold) accumulate_max(fft_data, power.max, n); else s31_power_acc(fft_data, power.sum, n / 8);
            uint32_t c3 = esp_cpu_get_cycle_count();
            prof_unpack += c1 - c0; prof_fft += c2 - c1; prof_acc += c3 - c2; prof_blocks++;
        }
        ffts += blocks; snapshots++; samples += blocks * n;
        /* Nobody is fetching (paused UI, open keypad): restart instead of
         * letting the sums grow without bound. */
        if (ffts >= 480 && !atomic_load(&fetch_request)) {
            memset(&power, 0, n * sizeof(uint64_t));
            ffts = snapshots = busy_us = 0; samples = 0;
            interval_start = esp_timer_get_time();
        }
        int64_t t1 = esp_timer_get_time();
        busy_us += (unsigned)(t1 - t0);
        /* The UI core is halted while the dump gate is open. At low sample
         * rates a snapshot lasts milliseconds, so keep that below half of
         * the time; coverage stays far above the fast rates anyway. */
        unsigned stalled_us = elapsed;
        if ((int64_t)stalled_us * 2 > t1 - t0) {
            int64_t resume = t0 + 2 * (int64_t)stalled_us;
            while (esp_timer_get_time() < resume) vTaskDelay(1);
            limiter_total_us += (unsigned)(esp_timer_get_time() - t1);
        }
        capture_total_us += (unsigned)(t_captured - t0);
        process_total_us += (unsigned)(t1 - t_captured);
        if (atomic_load(&fetch_request) && ffts) {
            respond_total_us += (unsigned)(esp_timer_get_time() - atomic_load(&fetch_requested_at)); responses++;
            publish(&c, ffts, snapshots, t1 - interval_start, busy_us, samples);
            ffts = snapshots = busy_us = 0; samples = 0;
            interval_start = t1;
            atomic_store(&fetch_request, false);
            xSemaphoreGive(ready);
        }
    }
}

void analyzer_start(void) {
    ready = xSemaphoreCreateBinary();
    published = heap_caps_calloc(ANALYZER_NFFT_MAX, sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    natural_index = heap_caps_calloc(ANALYZER_NFFT_MAX, sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(published && natural_index);
    ESP_ERROR_CHECK(dsps_fft2r_init_sc16(twiddles, ANALYZER_NFFT_MAX));
    worker = xTaskCreateStaticPinnedToCore(worker_task, "analyzer", sizeof(task_stack) / sizeof(StackType_t),
                                           NULL, 20, task_stack, &task_tcb, 1);
    assert(worker);
}

void analyzer_set_config(const analyzer_config_t *config) {
    pending = *config;
    atomic_fetch_add(&config_generation, 1);
}

void analyzer_set_paused(bool value) { atomic_store(&paused, value); }

void analyzer_wait_idle(void) {
    while (!atomic_load(&idle)) vTaskDelay(1);
}

bool analyzer_fetch(float *db, unsigned *nfft, analyzer_stats_t *stats, TickType_t timeout) {
    atomic_store(&fetch_requested_at, esp_timer_get_time());
    atomic_store(&fetch_request, true);
    if (xSemaphoreTake(ready, timeout) != pdTRUE) return false;
    *nfft = published_nfft;
    memcpy(db, published, published_nfft * sizeof(float));
    *stats = published_stats;
    return true;
}
