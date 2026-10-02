/* Shared continuous ADC-dump capture: bank rotation/live reads and on-chip
 * spectrum reduction. Rotation scheme after h0m3us3r/eSpDR (capture.c),
 * adapted to one core under ESP-IDF. */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "spectrum_dc.h"

#if CONFIG_IDF_TARGET_ESP32C3
#define RING_BANKS 1u
#define RING_BANK_BASE 0x3fcb0000u
#define RING_BANK_END 0x3fcc0000u
#elif CONFIG_IDF_TARGET_ESP32C6
#define RING_BANKS 2u
#define RING_BANK_BASE 0x40820000u
#define RING_BANK_END 0x40860000u
#define RING_BANK_STRIDE 0x20000u
#elif CONFIG_IDF_TARGET_ESP32C61
#define RING_BANKS 2u
#define RING_BANK_BASE 0x40820000u
#define RING_BANK_END 0x40840000u
#else
#define RING_BANKS 3u
#define RING_BANK_BASE 0x3fcb0000u   /* banks 0..2; bank 3 holds ROM data */
#define RING_BANK_END 0x3fce0000u
#endif
#ifndef RING_BANK_STRIDE
#define RING_BANK_STRIDE 0x10000u
#endif
#ifndef RING_PAIRS
#define RING_PAIRS 16384u
#define RING_THRESHOLD 12288u        /* switch banks after this many pairs */
#endif
#if !CONFIG_IDF_TARGET_ESP32S3
#define RING_SPEC_NFFT_MAX 256u
#else
#define RING_SPEC_NFFT_MAX 2048u         /* SPEC FFT sizes: 256, 512, 1024, 2048 */

#endif

typedef enum {
    RING_OK = 0,
    RING_FAIL_ARG,
    RING_FAIL_LATE,      /* switch detected too late: detail = pairs written */
    RING_FAIL_AGE,       /* bank older than the ring allows: detail = cycles */
    RING_FAIL_START,     /* unit start not where the previous unit ended */
    RING_FAIL_END,       /* unit end not inside its sentinel window */
    RING_FAIL_LENGTH,    /* implausible unit length: detail = pairs */
    RING_FAIL_TRANSPORT, /* SPEC requested over UART */
} ring_status_t;

typedef enum { RING_MODE_STATS, RING_MODE_CAPTURE, RING_MODE_SPEC, RING_MODE_IQ } ring_mode_t;

typedef struct {
    ring_mode_t mode;
    unsigned rate;             /* esp-sdr rate code: 0 = 80, 1 = 40, 6 = 16 Msps */
    uint32_t duration_ms;      /* 0: run until the host sends any byte */
    unsigned capture_units;    /* CAPTURE: consecutive units, 1..RING_BANKS */
    unsigned nfft;             /* SPEC: FFT size (256, 512, 1024, 2048) */
    unsigned stride;           /* SPEC: FFT every stride-th nfft-pair block */
    unsigned units_per_frame;  /* SPEC: units merged into one output frame */
    bool max_hold;             /* SPEC: per-bin max instead of mean power */
    bool stats;                /* SPEC: insert SPS1 statistics frames (~4/s) */
    unsigned iq_dec;           /* IQ: decimation 64..1024 (power of two), two-stage FIR */
    unsigned iq_bits;          /* IQ: 4, 8 or 16 bits per component */
    unsigned iq_shift;         /* IQ: rounding right shift of the FIR output (10-bit sample * 32) */
    bool iq_rot;               /* IQ: shift by +fs/4 before the FIR (LO tuned fs/4 below) */
} ring_config_t;

typedef struct {
    uint16_t bank, first;
    uint32_t count;
} ring_unit_t;

typedef struct {
    uint32_t status, detail;
    uint32_t units;
    uint64_t pairs;
    uint64_t elapsed_us;
    uint32_t late_max;   /* pairs past the threshold when the switch happened */
    uint32_t work_max;   /* longest single processing slice, CPU cycles */
    uint32_t frames, drops, abandoned, ffts;
    bool stopped_by_host;
    ring_unit_t cap[RING_BANKS];
} ring_result_t;

void ring_capture_init(void);
/* Caller has prepared the receiver (tuning, gain, filter). Interrupts are
 * disabled throughout bank rotation; C3 masks only its sample copy. */
void ring_capture_run(const ring_config_t *config, ring_result_t *result);
const uint32_t *ring_capture_bank(unsigned bank);
unsigned ring_capture_rate_hz(unsigned rate);
bool ring_capture_valid_nfft(unsigned n);
/* 0 Hz handling after the FFT: 0 = notch bin 0, 1 = slow DC tracker (default). */
#define ring_capture_dc_mode spectrum_dc_mode
#if CONFIG_IDF_TARGET_ESP32S3
/* Second core as SPEC worker (started by ring_capture_init when available). */
bool ring_capture_core1_alive(void);
bool ring_capture_dual_active(void);
void ring_capture_set_dual(bool on);
extern bool ring_capture_assist;     /* core 0 helps core 1 between bank switches */
extern uint32_t ring_capture_c0_blocks; /* blocks core 0 handed off in the last run */
#endif
