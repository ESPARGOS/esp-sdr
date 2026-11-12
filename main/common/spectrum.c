/* Portable on-chip spectra for targets without a validated continuous backend.
 * Each frame is an independent snapshot. Frame indices use elapsed wall time,
 * and flag bit 3 explicitly marks the gaps between acquisitions. */
#include "spectrum.h"
#include "spectrum_dc.h"
#include "spectrum_math.h"
#include "spectrum_fft.h"
#include "spectrum_stats.h"
#include "esp_cpu.h"
#include "burst_serial.h"
#include "dsps_fft2r.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_FFT      2048u
#define HEADER_BYTES 28u
#if CONFIG_IDF_TARGET_ESP32C2
/* Quiescent during RF capture; keep heap/stacks in the other SRAM banks. */
#define SPEC_STORAGE __attribute__((section(".c2_spectrum"), aligned(16)))
#else
#define SPEC_STORAGE
#endif
#if CONFIG_IDF_TARGET_ESP32C5
/* The finite writer uses bank 1's upper 64 KiB (0x40830000). Reuse its
 * reserved lower half for snapshot DSP/output, accessed only after capture
 * restores CPU ownership. The continuous engine overwrites this workspace
 * with samples and uses its own small BSS workspace instead. */
static int16_t *const fft_data = (int16_t *)0x40820000;
static int16_t *const window = (int16_t *)0x40822000;
static union {
    float *mean;
    uint32_t *maximum;
} powers = {.mean = (float *)0x40823000};
static uint8_t *const frame = (uint8_t *)0x40825000;
_Static_assert(0x40825000u + HEADER_BYTES + MAX_FFT + 4 <= 0x40830000u,
               "C5 snapshot workspace overlaps finite capture");
#else
static int16_t fft_data[2 * MAX_FFT] __attribute__((aligned(16))) SPEC_STORAGE;
static int16_t window[MAX_FFT] SPEC_STORAGE;
/* Same workspace size for both detectors. Max-hold never needs floats;
 * mean accumulation retains fractional precision without extra SRAM. */
static union {
    float mean[MAX_FFT];
    uint32_t maximum[MAX_FFT];
} powers SPEC_STORAGE;
static uint8_t frame[HEADER_BYTES + MAX_FFT + 4] SPEC_STORAGE;
#endif
static unsigned setup_n;
static bool ready;

spectrum_workspace_t spectrum_workspace(void) {
    setup_n = 0;
    return (spectrum_workspace_t){fft_data, window, powers.mean, frame};
}

static int16_t twiddles[MAX_FFT] __attribute__((aligned(16))) SPEC_STORAGE;
bool spectrum_fft_init(void) {
    if (!ready) ready = dsps_fft2r_init_sc16(twiddles, MAX_FFT) == ESP_OK;
    return ready;
}

static const unsigned rates[] = {
#if CONFIG_IDF_TARGET_ESP32H2
    0,        0,        0,        0,      0, 0,
    16000000, 32000000, 10666667, 6400000
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C2 ||                   \
    CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
    80000000, 40000000, 0,       0,
    0,        0,        16000000
#else
    80000000, 40000000, 20000000, 10000000,
    8000000,  4000000
#endif
};
static unsigned rate_hz(unsigned code) {
    return code < sizeof(rates) / sizeof(rates[0]) ? rates[code] : 0;
}
static bool send_text(const char *s) {
    return burst_serial_send(s, strlen(s));
}
static void put16(unsigned at, uint16_t v) {
    frame[at] = v;
    frame[at + 1] = v >> 8;
}
static void put32(unsigned at, uint32_t v) {
    put16(at, v);
    put16(at + 2, v >> 16);
}
static void put64(unsigned at, uint64_t v) {
    put32(at, v);
    put32(at + 4, v >> 32);
}
static int16_t clamp16(int32_t v) {
    return v > 32767 ? 32767 : v < -32768 ? -32768 : v;
}

static void capabilities(void) {
    send_text("SPECINFO {\"continuous\":false,\"transports\":["
#if CONFIG_IDF_TARGET_ESP32C2
              "\"UART\""
#else
              "\"USB\",\"UART\""
#endif
              "],\"profiles\":[");
    bool first = true;
    for (unsigned r = 0; r < sizeof(rates) / sizeof(rates[0]); r++)
        if (rates[r]) {
            for (unsigned n = 256; n <= MAX_FFT; n *= 2) {
                char text[96];
                snprintf(text, sizeof(text), "%s[%u,%u,%u,1,1,%u]", first ? "" : ",", rates[r], r, n,
#if CONFIG_IDF_TARGET_ESP32C5
                         n == 256 && r >= 4 && burst_serial_port() != BURST_SERIAL_UART
#elif CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
                         n == 256 && burst_serial_port() != BURST_SERIAL_UART
#else
                         0u
#endif
                );
                send_text(text);
                first = false;
            }
        }
    send_text("]}\n");
}

typedef struct {
    size_t pending, sent;
    uint32_t *frames;
    int64_t progress;
} spectrum_output_t;
static void pump_spectrum(void *context) {
    spectrum_output_t *out = context;
    if (!out->pending) return;
    size_t written = burst_serial_try_send(frame + out->sent, out->pending - out->sent);
    out->sent += written;
    if (written) out->progress = esp_timer_get_time();
    if (out->sent == out->pending) {
        out->pending = out->sent = 0;
        (*out->frames)++;
    }
}

/* Encoding consumes the completed batch; transmission owns frame[] while
 * the next batch accumulates independently in powers. */
static size_t pack_spectrum(unsigned n, unsigned log2n, unsigned det, unsigned merged, unsigned frames,
                            uint64_t index, uint8_t gain) {
    memcpy(frame, "SPC1", 4);
    put32(4, frames);
    put64(8, index);
    put32(16, n * merged);
    put16(20, merged);
    frame[22] = 8 | (det ? 1 : 0);
    frame[23] = gain;
    put16(24, 0);
    frame[26] = log2n;
    frame[27] = 2;
    const float mean_scale = 1.0f / merged;
    for (unsigned j = 0, bin = 0; j < n; j++) {
        frame[HEADER_BYTES + bin] = det ? spectrum_power_code(powers.maximum[j])
                                        : spectrum_mean_power_code(powers.mean[j] * mean_scale);
        unsigned bit = n >> 1;
        while (bin & bit) {
            bin ^= bit;
            bit >>= 1;
        }
        bin ^= bit;
    }
    put32(HEADER_BYTES + n, esp_rom_crc32_le(0, frame, HEADER_BYTES + n));
    memset(powers.mean, 0, n * sizeof(float));
    return HEADER_BYTES + n + 4;
}

bool spectrum_command(const char *line, unsigned frequency_mhz, spectrum_acquire_fn acquire) {
    unsigned dc_mode;
    char dc_extra;
    if (!strcmp(line, "DC?")) {
        char text[16];
        snprintf(text, sizeof(text), "DC %u\n", spectrum_dc_mode);
        send_text(text);
        return true;
    }
    if (sscanf(line, "DC %u %c", &dc_mode, &dc_extra) == 1 && dc_mode < 2) {
        spectrum_dc_mode = dc_mode;
        send_text("OK\n");
        return true;
    }
    if (!strcmp(line, "SPECINFO?")) {
        capabilities();
        return true;
    }
    if (strncmp(line, "SPEC ", 5)) return false;
    unsigned ms, stride, units, det, rate, n, stats = 0;
    char extra;
    int fields =
        sscanf(line, "SPEC %u %u %u %u %u %u %u %c", &ms, &stride, &units, &det, &rate, &n, &stats, &extra);
    if ((fields != 6 && fields != 7) || stats > 1 || ms > 86400000u || stride != 1 || !units || units > 8 ||
        det > 1 || !rate_hz(rate) || n < 256 || n > MAX_FFT || (n & (n - 1))) {
        send_text("ERR spec_args\n");
        return true;
    }
#if CONFIG_IDF_TARGET_ESP32C5
    setup_n = 0; /* A preceding continuous run reused the snapshot window. */
#endif
    spectrum_fft_init();
    if (!ready) {
        send_text("ERR spectrum_memory\n");
        return true;
    }
    if (setup_n != n) {
        for (unsigned j = 0; j < n; j++) window[j] = (int16_t)lrintf(16383.5f * (1 - cosf(2 * M_PI * j / n)));
        setup_n = n;
    }
    unsigned log2n = 0;
    while ((1u << log2n) < n) log2n++;
    char text[192];
    snprintf(text, sizeof(text), "SPEC %u %u %u %u\n", n, rate_hz(rate), n, frequency_mhz);
    if (!send_text(text)) return true;
    int64_t start = esp_timer_get_time(), yield_at = start;
    uint32_t frames = 0, ffts = 0, status = 0;
    uint64_t pairs = 0;
    bool stopped = false;
    spectrum_dc_t dc = {0};
    spectrum_stats_t telemetry;
    spectrum_stats_init(&telemetry);
    unsigned merged = 0;
    uint8_t gain = 0;
    uint64_t index = 0;
    spectrum_output_t out = {.frames = &frames, .progress = start};
    memset(powers.mean, 0, n * sizeof(float));
    do {
        if (burst_serial_stop_requested()) {
            stopped = true;
            break;
        }
        uint32_t busy_start = esp_cpu_get_cycle_count();
        if (!merged) index = (uint64_t)(esp_timer_get_time() - start) * rate_hz(rate) / 1000000u;
        const uint32_t *words;
        unsigned elapsed;
        if (!acquire(n, rate, &words, &elapsed)) {
            status = 1;
            break;
        }
        if (!merged) gain = words[0] >> 20;
        for (unsigned j = 0; j < n; j++) {
            int32_t i = ((int32_t)(words[j] << 22) >> 22), q = ((int32_t)(words[j] << 12) >> 22);
            fft_data[2 * j] = clamp16((i * window[j]) >> 9);
            fft_data[2 * j + 1] = clamp16((q * window[j]) >> 9);
        }
        spectrum_fft_poll(fft_data, n, dsps_fft_w_table_sc16, pump_spectrum, &out);
        spectrum_dc_apply(&dc, fft_data, n);
        for (unsigned j = 0; j < n; j++) {
            uint32_t p = spectrum_complex_power(fft_data[2 * j], fft_data[2 * j + 1]);
            if (det) {
                if (p > powers.maximum[j]) powers.maximum[j] = p;
            } else
                powers.mean[j] += (float)p;
        }
        ffts++;
        pairs += n;
        merged++;
        pump_spectrum(&out);
        if (out.pending && esp_timer_get_time() - out.progress > 3000000) {
            status = 7;
            break;
        }
        if (!out.pending) {
            if (stats) spectrum_stats_emit(&telemetry, n, rate_hz(rate), ffts, 0, 0, 0, 0, burst_serial_send);
            out.pending = pack_spectrum(n, log2n, det, merged, frames, index, gain);
            merged = 0;
            out.progress = esp_timer_get_time();
            pump_spectrum(&out);
        }
        telemetry.busy += (uint32_t)(esp_cpu_get_cycle_count() - busy_start);
        if (merged == 65535) break; /* bound the wire FFT count if the host stalls */
        if (esp_timer_get_time() - yield_at >= 20000) {
            vTaskDelay(1);
            yield_at = esp_timer_get_time();
        }
    } while (!ms || esp_timer_get_time() - start < (int64_t)ms * 1000);
    /* Drain in order; a partial final batch never contains an earlier frame. */
    if (out.pending) {
        if (burst_serial_send(frame + out.sent, out.pending - out.sent))
            frames++;
        else
            status = 7;
    }
    if (!status && merged) {
        size_t size = pack_spectrum(n, log2n, det, merged, frames, index, gain);
        if (burst_serial_send(frame, size))
            frames++;
        else
            status = 7;
    }
    snprintf(text, sizeof(text), "SPECEND %u 0 %u %" PRIu64 " %" PRIi64 " 0 0 %u 0 0 %u %u\n",
             (unsigned)status, (unsigned)ffts, pairs, esp_timer_get_time() - start, (unsigned)frames,
             (unsigned)ffts, stopped);
    send_text(text);
    return true;
}
