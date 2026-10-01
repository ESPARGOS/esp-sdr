/* ESP32-S3 continuous ADC-dump ring: gapless bank rotation plus on-chip
 * spectrum reduction. Rotation scheme after h0m3us3r/eSpDR (capture.c),
 * adapted to one core under ESP-IDF. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define S3_RING_BANKS 3u
#define S3_RING_BANK_BASE 0x3fcb0000u   /* banks 0..2; bank 3 holds ROM data */
#define S3_RING_BANK_END 0x3fce0000u
#define S3_RING_PAIRS 16384u
#define S3_RING_THRESHOLD 12288u        /* switch banks after this many pairs */
#define S3_SPEC_NFFT_MAX 2048u         /* SPEC FFT sizes: 256, 1024, 2048 */

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

typedef enum { RING_MODE_STATS, RING_MODE_CAPTURE, RING_MODE_SPEC } ring_mode_t;

typedef struct {
    ring_mode_t mode;
    unsigned rate;             /* esp-sdr rate code: 0 = 80, 1 = 40, 6 = 16 Msps */
    uint32_t duration_ms;      /* 0: run until the host sends any byte */
    unsigned capture_units;    /* CAPTURE: consecutive units, 1..S3_RING_BANKS */
    unsigned nfft;             /* SPEC: FFT size (256, 1024, 2048) */
    unsigned stride;           /* SPEC: FFT every stride-th nfft-pair block */
    unsigned units_per_frame;  /* SPEC: units merged into one output frame */
    bool max_hold;             /* SPEC: per-bin max instead of mean power */
    bool stats;                /* SPEC: insert SPS1 statistics frames (~4/s) */
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
    ring_unit_t cap[S3_RING_BANKS];
} ring_result_t;

void s3_ring_init(void);
/* Caller has prepared the receiver (tuning, gain, filter). Interrupts are
 * disabled for the whole run; USB Serial/JTAG is driven directly. */
void s3_ring_run(const ring_config_t *config, ring_result_t *result);
const uint32_t *s3_ring_bank(unsigned bank);
unsigned s3_ring_rate_hz(unsigned rate);
bool s3_ring_valid_nfft(unsigned n);
/* Second core as SPEC worker (started by s3_ring_init when available). */
bool s3_ring_core1_alive(void);
bool s3_ring_dual_active(void);
void s3_ring_set_dual(bool on);
extern bool s3_ring_assist;
extern uint32_t s3_ring_c0_blocks;
extern unsigned s3_ring_dc_mode; /* 0 = notch bin 0, 1 = slow DC tracker */
