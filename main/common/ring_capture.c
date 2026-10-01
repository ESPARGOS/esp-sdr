/*
 * Continuous RF capture: S3/C6/C61 bank rotation and C3 live-bank reads.
 *
 * The ADC dump engine writes IQ pairs at a ring index that advances
 * continuously (mod 16384) into whichever of the SRAM banks is selected.
 * Switching the selected bank while the engine runs therefore splits the
 * sample stream into gapless "units", one per bank visit. The scheme follows
 * h0m3us3r/eSpDR capture.c, reduced to one core and three banks:
 *
 *   - while the writer fills bank b, bank b+1 gets sentinel words where the
 *     next unit is predicted to start and to end;
 *   - once THRESHOLD pairs are in bank b, the writer is moved to bank b+1;
 *   - the exact first and last pair of the finished unit are located by
 *     binary search over the sentinel windows, and each unit's first pair
 *     must equal the previous unit's end, which proves the stream gapless.
 *
 * Between polls the CPU runs short work slices (one 256-point FFT) on units
 * already finished, so the switch latency is bounded by one slice. When the
 * processing falls behind, the unfinished rest of the oldest unit is
 * abandoned instead of stalling the ring (counted in `abandoned`).
 *
 * Bank-rotation targets keep interrupts disabled for the run: no FreeRTOS tick, no Wi-Fi
 * ISR, no driver. Output goes straight into the USB Serial/JTAG FIFO from a
 * RAM queue; a full queue drops whole frames (counted, never blocks).
 * Those targets require CONFIG_ESP_INT_WDT=n. C3 masks interrupts only
 * for copying one FFT window, and yields during processing.
 */
#include "ring_capture.h"
#include "spectrum.h"
#include "sdkconfig.h"

#include <math.h>
#include <string.h>

#include "dsps_fft2r.h"
/* esp-dsp aes3 FFT with the sum-branch bias corrected (s3_fft_rnd.S). */
extern int16_t *dsps_fft_w_table_sc16;
int s3_fft2r_sc16_rnd(int16_t *data, int N, int16_t *w);
#if CONFIG_IDF_TARGET_ESP32S3
#define S3_FFT(buf, n) s3_fft2r_sc16_rnd((buf), (int)(n), dsps_fft_w_table_sc16)
#else
#define S3_FFT(buf, n) dsps_fft2r_sc16_ansi((buf), (n))
#endif
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ring_io.h"
#include "soc/soc.h"
#include "esp_heap_caps.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "hal/cpu_utility_ll.h"
#include "xt_utils.h"
#include "esp32s3/rom/ets_sys.h"
#endif

/* Placement of the ring control code (flash by default; see notes). */
#ifndef RING_HOT
#define RING_HOT
#endif
#define RING_MASK (RING_PAIRS - 1u)
#define THRESHOLD RING_THRESHOLD
#define SENTINEL 0xA5C33C5Au
#if !CONFIG_IDF_TARGET_ESP32S3
#define START_GUARD 1024u
#define END_GUARD 2048u
#define LATE_LIMIT 800u
#else
#define START_GUARD 3072u  /* sentinels from a unit's earliest possible start */
#define END_GUARD 4096u    /* sentinels from a unit's earliest possible end */
#define LATE_LIMIT 2000u   /* switch lateness allowed: 2*LATE_LIMIT < END_GUARD */
#endif
#define UNIT0_END_GUARD 1024u /* unit 0: whole bank is sentinel, stay clear of its start */
#define MIN_PAIRS THRESHOLD
#define MAX_PAIRS (THRESHOLD + LATE_LIMIT + 64u)
#define NOT_FOUND 0xffffffffu

#if CONFIG_IDF_TARGET_ESP32C3
#define DUMP_CTRL_REG 0x60033d5cu
#define DUMP_WRITE_INDEX_REG 0x60033d60u
#define DUMP_CONFIG_REG 0x60033d90u
#define DUMP_BANK_SELECT_REG 0x600c1020u
#define DUMP_CTRL_RUN 0x80000000u
#define DUMP_CTRL_CIRCULAR 0x00024000u
#define DUMP_CONFIG_IQ 0x000c2040u
#elif !CONFIG_IDF_TARGET_ESP32S3
#define DUMP_CTRL_REG 0x600a9004u
#define DUMP_WRITE_INDEX_REG 0x600a9008u
#if CONFIG_IDF_TARGET_ESP32C6
#define DUMP_CONFIG_REG 0x600a9014u
#else
#define DUMP_CONFIG_REG 0x600a9018u
#endif
#define DUMP_BANK_SELECT_REG 0x60095004u
#define DUMP_CTRL_RUN 0x80000000u
#define DUMP_CTRL_CIRCULAR 0x00024000u
#if CONFIG_IDF_TARGET_ESP32C6
#define DUMP_CONFIG_IQ ((1u<<6)|(2u<<12)|(3u<<18))
#else
#define DUMP_CONFIG_IQ (24u|(25u<<6)|(26u<<12)|(27u<<18)|(1u<<24))
#endif
#else
#define DUMP_CTRL_REG 0x60033d5cu
#define DUMP_WRITE_INDEX_REG 0x60033d60u
#define DUMP_CONFIG_REG 0x60033d90u
#define DUMP_BANK_SELECT_REG 0x600c101cu
#define DUMP_CTRL_RUN 0x80000000u
#define DUMP_CTRL_CIRCULAR 0x00024000u /* circular 16384-pair ring, IQ source 0 */
#define DUMP_CONFIG_IQ 0x000c2040u
#endif

static inline void select_banks(uint32_t saved, unsigned mask) {
#if !CONFIG_IDF_TARGET_ESP32S3
    #if CONFIG_IDF_TARGET_ESP32C6
    REG_WRITE(DUMP_BANK_SELECT_REG,(saved & ~0x10f00u)|(mask<<9));
#else
    REG_WRITE(DUMP_BANK_SELECT_REG,(saved & ~0x11f00u)|(mask<<10));
#endif
    __asm__ volatile("fence rw,rw" ::: "memory");
    (void)REG_READ(DUMP_BANK_SELECT_REG);
#else
    REG_WRITE(DUMP_BANK_SELECT_REG,(saved & ~15u)|mask);
#endif
}

_Static_assert(RING_BANK_BASE + RING_BANKS * RING_BANK_STRIDE == RING_BANK_END, "bank map");
_Static_assert(START_GUARD + THRESHOLD <= RING_PAIRS, "start window overlaps");
_Static_assert(END_GUARD + THRESHOLD <= RING_PAIRS, "end window overlaps unit start");
_Static_assert(2u * LATE_LIMIT + 64u < END_GUARD && LATE_LIMIT + 64u < START_GUARD, "guards");

static inline uint32_t *bank_ptr(unsigned b) {
    return (uint32_t *)(RING_BANK_BASE + b * RING_BANK_STRIDE);
}
const uint32_t *ring_capture_bank(unsigned b) { return bank_ptr(b); }

unsigned ring_capture_rate_hz(unsigned rate) {
    #if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    const unsigned rates[]={80000000,40000000,20000000,10000000,8000000,4000000};
    return rate<6?rates[rate]:0;
#else
    return rate == 6 ? 16000000u : rate == 1 ? 40000000u : 80000000u;
#endif
}
static uint32_t rate_bits(unsigned rate) {
    #if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
    return 0;
#else
    return rate == 6 ? (1u << 16) : rate == 1 ? (1u << 15) : 0;
#endif
}
static unsigned cycles_per_pair(unsigned rate) { /* 240 MHz CPU */
    return CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ*1000000u/ring_capture_rate_hz(rate);
}

/* ---- sentinel windows ------------------------------------------------- */

/* Fills ring indices [at, at+count) of bank b with sentinels, wrapping.
 * 128-bit PIE stores (from eSpDR capture.c, 0BSD): at 80 Msps the scalar
 * loop was too slow to prepare 7 k words before the next switch. */
RING_HOT static void fill_sentinels(unsigned b, unsigned at, unsigned count) {
    at &= RING_MASK;
    while (count) {
        unsigned n = RING_PAIRS - at;
        if (n > count) n = count;
        uint32_t *p = bank_ptr(b) + at;
        unsigned head = (-at) & 3u;
        if (head > n) head = n;
        for (unsigned i = 0; i < head; i++) *p++ = SENTINEL;
        unsigned vectors = 0;
#if CONFIG_IDF_TARGET_ESP32S3
        vectors = (n - head) / 4;
        if (vectors) {
            uint32_t value = SENTINEL;
            __asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
                             "ee.movi.32.q q6, %[v], 1\n"
                             "ee.movi.32.q q6, %[v], 2\n"
                             "ee.movi.32.q q6, %[v], 3\n"
                             "loopnez %[n], 1f\n"
                             "ee.vst.128.ip q6, %[p], 16\n"
                             "1:"
                             : [p] "+a"(p)
                             : [v] "a"(value), [n] "a"(vectors)
                             : "memory");
        }
#endif
        for (unsigned i = head + vectors * 4; i < n; i++) *p++ = SENTINEL;
        count -= n;
        at = 0;
    }
#if CONFIG_IDF_TARGET_ESP32S3
    __asm__ volatile("memw" ::: "memory");
#else
    __asm__ volatile("fence rw,rw" ::: "memory");
#endif
}

/* First written pair in [origin, origin+guard), which holds sentinels up to
 * the unit start and data from there on. */
RING_HOT static uint32_t find_first(unsigned b, unsigned origin, unsigned guard) {
    const uint32_t *p = bank_ptr(b);
    if (p[origin & RING_MASK] != SENTINEL) return NOT_FOUND; /* started early */
    if (p[(origin + guard - 1) & RING_MASK] == SENTINEL) return NOT_FOUND;
    unsigned lo = 0, hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & RING_MASK] == SENTINEL) lo = mid + 1;
        else hi = mid;
    }
    return (origin + lo) & RING_MASK;
}

/* One past the last written pair in [origin, origin+guard), searching from
 * the write index seen before the switch. */
RING_HOT static uint32_t find_end(unsigned b, unsigned origin, unsigned write_index, unsigned guard) {
    const uint32_t *p = bank_ptr(b);
    unsigned lo = (write_index - origin) & RING_MASK;
    if (lo >= guard || p[(origin + guard - 1) & RING_MASK] != SENTINEL) return NOT_FOUND;
    unsigned hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & RING_MASK] == SENTINEL) hi = mid;
        else lo = mid + 1;
    }
    return (origin + lo) & RING_MASK;
}

/* ---- USB Serial/JTAG output queue ------------------------------------- */

#if !CONFIG_IDF_TARGET_ESP32S3
#define TXQ_SIZE 2048u
#else
#define TXQ_SIZE 16384u /* 7 frames of 2048 bins, 56 of 256 */
#endif
static uint8_t txq[TXQ_SIZE];
static uint32_t txq_head, txq_tail; /* free-running byte counters */
_Static_assert((TXQ_SIZE & (TXQ_SIZE - 1u)) == 0, "queue size");

RING_HOT static bool txq_push(const void *data, size_t n) {
    if (TXQ_SIZE - (txq_head - txq_tail) < n) return false;
    const uint8_t *s = data;
    uint32_t off = txq_head & (TXQ_SIZE - 1u), first = TXQ_SIZE - off;
    if (first > n) first = n;
    memcpy(txq + off, s, first);
    memcpy(txq, s + first, n - first);
    txq_head += n;
    return true;
}

/* At most one 64-byte packet per call, never waits. */
RING_HOT static void txq_pump(void) {
    uint32_t used = *(volatile uint32_t *)&txq_head - txq_tail; /* core 1 pushes too */
    if (!used) return;
    uint32_t off = txq_tail % TXQ_SIZE, n = TXQ_SIZE - off;
    if (n > used) n = used;
    if (n > 64) n = 64;
    int written = ring_write(txq + off,n);
    if (written > 0) txq_tail += (uint32_t)written;
}

RING_HOT static bool host_input(void) {
    if (!ring_input_available()) return false;
    uint8_t b;
    for (unsigned k = 0; k < 64 && ring_input_available(); k++) {
        if (ring_read_byte(&b) == 1 && b == '\n') break;
    }
    return true;
}

/* ---- spectrum reduction ------------------------------------------------ */

#define SPEC_MAGIC 0x31435053u /* "SPC1" */
#define CHUNK 512u              /* elements per unpack / accumulate / emit slice */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;      /* frame sequence number */
    uint64_t pair_index; /* stream index of the first pair (gapless clock) */
    uint32_t pairs;      /* stream pairs this frame spans */
    uint16_t ffts;       /* FFTs merged into this frame */
    uint8_t flags;       /* bit0 max-hold, bit1 work abandoned, bit2 frames dropped before */
    uint8_t gain;        /* bits 20..27 of the frame's first IQ word */
    uint16_t drops;      /* cumulative dropped frames, saturating */
    uint8_t nfft_log2;   /* frame carries 1 << nfft_log2 bin codes */
    uint8_t db_step;     /* bin code = 10*log10(power) * db_step */
} spec_header_t;
_Static_assert(sizeof(spec_header_t) == 28, "SPEC header layout");

static int16_t window_q15[RING_SPEC_NFFT_MAX];
#if CONFIG_IDF_TARGET_ESP32S3
/* Window with each coefficient twice (I and Q lanes) for the PIE unpack. */
static int16_t win2[2 * RING_SPEC_NFFT_MAX] __attribute__((aligned(16)));
void s3_unpack_iq10_win(const uint32_t *src, int16_t *dst, const int16_t *win2, unsigned groups8);
#endif
static int16_t fft_buf[2 * RING_SPEC_NFFT_MAX] __attribute__((aligned(16)));
/* Indexed by FFT output slot (bit-reversed order); emit maps slot -> bin.
 * mean: float power sum; max-hold: uint32 power per FFT slot (same storage). */
#if CONFIG_IDF_TARGET_ESP32S3
static float accum_buf[2][RING_SPEC_NFFT_MAX]; /* double buffer: core 1 encodes one while filling the other */
#else
static float accum_buf[1][RING_SPEC_NFFT_MAX];
#endif
static float *accum = accum_buf[0];
static uint16_t bin_of[RING_SPEC_NFFT_MAX]; /* FFT output slot -> natural-order bin */
static uint8_t frame_out[sizeof(spec_header_t) + RING_SPEC_NFFT_MAX + 4];
static unsigned spec_n, spec_log2;        /* FFT size of the current run */

static void log_tables_init(void);
#if CONFIG_IDF_TARGET_ESP32S3
static void c1_start(void);
#endif
static bool dsp_ready;
void ring_capture_init(void) {
    if (dsp_ready) return;
#if !CONFIG_IDF_TARGET_ESP32S3
    dsp_ready = spectrum_fft_init();
    log_tables_init();
#else
    /* twiddles from the heap: BSS must end below the RF ring (sram_guard.ld) */
    dsp_ready = dsps_fft2r_init_sc16(NULL,RING_SPEC_NFFT_MAX)==ESP_OK;
    log_tables_init();
    c1_start();
#endif
}

bool ring_capture_valid_nfft(unsigned n) { return n <= RING_SPEC_NFFT_MAX && (n == 256 || n == 1024 || n == 2048); }

/* Hann window and bit-reversal bin map for FFT size n; outside the timed run.
 * Scaling is independent of n: IQ10 << 6 in, the int16 FFT divides by n. */
static void spec_setup(unsigned n) {
    if (n == spec_n) return;
    unsigned log2n = 0;
    while ((1u << log2n) < n) log2n++;
    for (unsigned i = 0; i < n; i++) {
        window_q15[i] = (int16_t)lrintf(32767.0f * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / n)));
#if CONFIG_IDF_TARGET_ESP32S3
        win2[2 * i] = win2[2 * i + 1] = window_q15[i];
#endif
        unsigned r = 0;
        for (unsigned bit = 0; bit < log2n; bit++) r |= ((i >> bit) & 1u) << (log2n - 1 - bit);
        bin_of[i] = (uint16_t)r;
    }
    spec_n = n;
    spec_log2 = log2n;
}

/* dB coding without float math: code = 6.0206*log2(v) + 0.5 (0.5 dB steps),
 * taken from the IEEE-754 bits of v: exponent table + 7-bit mantissa table,
 * both in 1/16 code units. Max error 0.09 code (0.05 dB), better than the
 * former polynomial fast_log2. A mean frame divides by the FFT count, which in
 * the log domain is a constant offset per frame. */
static int16_t log_e_q4[256], log_m_q4[128];
static void log_tables_init(void) {
    for (unsigned e = 0; e < 256; e++) log_e_q4[e] = (int16_t)lrintf(16.0f * 6.0206f * ((float)e - 127.0f)) + 8;
    for (unsigned m = 0; m < 128; m++)
        log_m_q4[m] = (int16_t)lrintf(16.0f * 6.0206f * log2f(1.0f + ((float)m + 0.5f) / 128.0f));
}


/* Block pipeline, one slice per call so the ring is polled in between:
 * unpack (CHUNK samples, needs the bank) -> FFT (whole block) ->
 * accumulate (whole FFT, committed atomically). Only unpacking reads the bank. */
IRAM_ATTR static void spec_unpack(const uint32_t *p, unsigned at, unsigned from, unsigned to) {
#if CONFIG_IDF_TARGET_ESP32S3
    unsigned first = (at + from) & RING_MASK, count = to - from;
    if (!(from & 7u) && !(count & 7u) && count && first + count <= RING_PAIRS) {
        /* PIE path (~3 cycles/sample instead of ~17); bit-identical output */
        s3_unpack_iq10_win(p + first, fft_buf + 2 * from, win2 + 2 * from, count / 8u);
        return;
    }
#endif
    for (unsigned i = from; i < to; i++) {
        uint32_t w = p[(at + i) & RING_MASK];
        int32_t I = (int32_t)(w << 22) >> 22, Q = (int32_t)(w << 12) >> 22;
        fft_buf[2 * i] = (int16_t)((I * window_q15[i]) >> 9);     /* IQ10 << 6 */
        fft_buf[2 * i + 1] = (int16_t)((Q * window_q15[i]) >> 9);
    }
}

/* Remove the receiver's DC offset (zero-IF LO leakage, 30-36 dB over the floor)
 * after the FFT, for free: with the periodic Hann window a pure DC lands in bin 0
 * and, at -1/2 of it, in bins +-1. Zero bin 0 and add half of it back to +-1.
 * Output is bit-reversed: natural bin 0 is slot 0, bin 1 is slot n/2, bin n-1
 * is slot n-1. Same notch as the burst mode's block-mean removal, 1 bin wide. */
static inline int16_t sat16(int32_t v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v; }
/* DC handling after the FFT (slot order: slot 0 = bin 0, n/2 = bin +1,
 * n-1 = bin -1). A constant c under the Hann window gives X0 = cN/2 and
 * X+-1 = -cN/4, so removing d from X0 means adding d/2 to X+-1.
 * mode 1 (default): d = slow running average of X0 (static DC offset only,
 * signals near 0 Hz survive). mode 0: d = X0 (old notch). */
unsigned ring_capture_dc_mode = 1;
static volatile int32_t dc_re_q8, dc_im_q8; /* estimate x256, shared by both cores */
static volatile uint32_t dc_ok;
#define DC_SHIFT 6 /* 1/64 per block */
IRAM_ATTR static void dc_track(int16_t *x) {
    int32_t r0 = x[0], i0 = x[1], dr, di;
    if (ring_capture_dc_mode == 0) {
        dr = r0;
        di = i0;
    } else {
        int32_t er = dc_re_q8, ei = dc_im_q8;
        if (!dc_ok) {
            er = r0 * 256;
            ei = i0 * 256;
            dc_ok = 1;
        } else {
            er += (r0 * 256 - er) >> DC_SHIFT;
            ei += (i0 * 256 - ei) >> DC_SHIFT;
        }
        dc_re_q8 = er;
        dc_im_q8 = ei;
        dr = (er + 128) >> 8;
        di = (ei + 128) >> 8;
    }
    unsigned s1 = spec_n / 2, sm = spec_n - 1;
    x[0] = sat16(r0 - dr);
    x[1] = sat16(i0 - di);
    x[2 * s1] = sat16(x[2 * s1] + dr / 2);
    x[2 * s1 + 1] = sat16(x[2 * s1 + 1] + di / 2);
    x[2 * sm] = sat16(x[2 * sm] + dr / 2);
    x[2 * sm + 1] = sat16(x[2 * sm + 1] + di / 2);
}
IRAM_ATTR static void spec_remove_dc(void) { dc_track(fft_buf); }

IRAM_ATTR static void spec_accumulate(bool max_hold, unsigned from, unsigned to) {
    const int16_t *x = fft_buf + 2 * from;
    if (max_hold) {
        uint32_t *pk = (uint32_t *)accum;
        for (unsigned k = from; k < to; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            uint32_t power = (uint32_t)(re * re) + (uint32_t)(im * im); /* <= 2^31 */
            if (power > pk[k]) pk[k] = power;
        }
    } else {
        for (unsigned k = from; k < to; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            accum[k] += (float)((uint32_t)(re * re) + (uint32_t)(im * im));
        }
    }
}

/* ---- run ---------------------------------------------------------------- */

typedef struct {
    bool pending;
    uint32_t first, count, next_block, blocks;
    uint64_t index;
    uint8_t gain;
} work_t;

enum { BLK_IDLE, BLK_UNPACK, BLK_FFT, BLK_ACCUM };

static struct {
    const ring_config_t *cfg;
    ring_result_t *res;
    work_t work[RING_BANKS];
    unsigned fifo[RING_BANKS], fifo_len; /* banks with pending work, oldest first */
    /* frame being accumulated */
    unsigned frame_units, frame_ffts;
    uint32_t frame_pairs;
    uint64_t frame_index;
    uint8_t frame_gain, frame_flags;
    bool dropped;
    /* block in flight */
    unsigned phase, pos, blk_bank;
    uint32_t blk_at;
    /* frame emission in progress (has priority over new accumulation) */
    bool emitting;
    int64_t last_ok;  /* last frame accepted by the output queue */
    unsigned emit_pos;
    int32_t emit_off_q4; /* mean: 16*6.0206*log2(ffts), max-hold: 0 */
    spec_header_t emit_h;
} st;

#if CONFIG_IDF_TARGET_ESP32S3

/* ---- live statistics frame ("SPS1", opt-in) ------------------------------ */
#define STAT_MAGIC 0x31535053u /* "SPS1" */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t c0_pm, c1_pm;       /* core load over the period, per mille */
    uint16_t cov_pm, mode;       /* FFT coverage per mille; bit0 dual, bit1 assist */
    uint32_t heap_free, heap_largest; /* internal heap before the run, bytes */
    uint32_t abandoned, drops;   /* run totals */
    uint16_t late_max, txq_pm;   /* switch lateness (pairs), output queue fill */
    uint32_t ffts_per_s;
} spec_stats_t;
_Static_assert(sizeof(spec_stats_t) == 36, "SPS1 layout");
static volatile struct { uint32_t c0_busy, c1_busy; } ld; /* busy cycles, own core each */
static struct {
    bool started;
    uint32_t t0, c0, c1, ffts, pairs, pairs_done;
    uint32_t heap_free, heap_largest;
    uint16_t mode;
} sx;
static bool c1_txq_push(const uint8_t *d, uint32_t n);
static bool txq_push(const void *data, size_t n);

IRAM_ATTR static void stats_maybe(bool core1_side) {
    uint32_t now = esp_cpu_get_cycle_count(); /* producer's own clock */
    if (!sx.started) {
        sx.started = true;
        sx.t0 = now;
        sx.c0 = ld.c0_busy;
        sx.c1 = ld.c1_busy;
        sx.ffts = st.res->ffts;
        sx.pairs = sx.pairs_done;
        return;
    }
    uint32_t dt = now - sx.t0;
    if (dt < 60000000u) return; /* 250 ms at 240 MHz */
    uint32_t per_mille = dt / 1000u, ms = dt / 240000u;
    uint32_t c0 = ld.c0_busy, c1 = ld.c1_busy;
    ring_result_t *r = st.res;
    uint32_t df = r->ffts - sx.ffts, dp = sx.pairs_done - sx.pairs;
    spec_stats_t m;
    m.magic = STAT_MAGIC;
    uint32_t v = (c0 - sx.c0) / per_mille;
    m.c0_pm = (uint16_t)(v > 1000 ? 1000 : v);
    v = (c1 - sx.c1) / per_mille;
    m.c1_pm = (uint16_t)(v > 1000 ? 1000 : v);
    v = dp >= 1000u ? (df * spec_n) / (dp / 1000u) : 0;
    m.cov_pm = (uint16_t)(v > 1000 ? 1000 : v);
    m.mode = sx.mode;
    m.heap_free = sx.heap_free;
    m.heap_largest = sx.heap_largest;
    m.abandoned = r->abandoned;
    m.drops = r->drops;
    m.late_max = (uint16_t)(r->late_max > 65535 ? 65535 : r->late_max);
    m.txq_pm = (uint16_t)(((*(volatile uint32_t *)&txq_head - *(volatile uint32_t *)&txq_tail) * 1000u) / TXQ_SIZE);
    m.ffts_per_s = ms ? df * 1000u / ms : 0;
    uint8_t buf[sizeof(spec_stats_t) + 4];
    const uint8_t *mp = (const uint8_t *)&m;
    for (unsigned i = 0; i < sizeof(m); i++) buf[i] = mp[i];
    uint32_t crc = esp_rom_crc32_le(0, buf, sizeof(m));
    for (unsigned i = 0; i < 4; i++) buf[sizeof(m) + i] = (uint8_t)(crc >> (8u * i));
    if (core1_side) (void)c1_txq_push(buf, sizeof(buf));
    else (void)txq_push(buf, sizeof(buf));
    sx.t0 = now;
    sx.c0 = c0;
    sx.c1 = c1;
    sx.ffts = r->ffts;
    sx.pairs = sx.pairs_done;
}

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

#if !CONFIG_IDF_TARGET_ESP32S3
#include "ring_scalar.h"
#endif

/* Close the accumulating frame; its bins are encoded by emit_chunk(). */
RING_HOT static void emit_chunk(void);
RING_HOT static void frame_close(void) {
    while (st.emitting) emit_chunk(); /* rare: previous frame not yet out */
    ring_result_t *r = st.res;
    if (!st.frame_ffts) {
        r->drops++;
        st.dropped = true;
        st.frame_units = st.frame_pairs = st.frame_flags = 0;
        return;
    }
    st.emit_h = (spec_header_t){
        .magic = SPEC_MAGIC, .frame = r->frames + r->drops, .pair_index = st.frame_index,
        .pairs = st.frame_pairs, .ffts = (uint16_t)st.frame_ffts,
        .flags = (uint8_t)((st.cfg->max_hold ? 1 : 0) | st.frame_flags | (st.dropped ? 4 : 0)),
        .gain = st.frame_gain, .drops = (uint16_t)(r->drops > 65535 ? 65535 : r->drops),
        .nfft_log2 = (uint8_t)spec_log2, .db_step = 2,
    };
    st.emit_off_q4 = (st.cfg->max_hold || st.frame_ffts < 2) ? 0
                   : (int32_t)lrintf(16.0f * 6.0206f * log2f((float)st.frame_ffts));
    st.emit_pos = 0;
    st.emitting = true;
    st.frame_units = st.frame_ffts = st.frame_pairs = 0;
    st.frame_flags = 0;
}

RING_HOT static void emit_chunk(void) {
    uint8_t *o = frame_out + sizeof(spec_header_t);
    unsigned end = st.emit_pos + CHUNK < spec_n ? st.emit_pos + CHUNK : spec_n;
    const int32_t off = st.emit_off_q4;
    uint32_t *a = (uint32_t *)accum; /* slot order; float bits (mean) or uint32 (max) */
    const bool max_hold = st.cfg->max_hold;
    for (unsigned k = st.emit_pos; k < end; k++) {
        uint32_t u = a[k];
        if (max_hold) {
            union { float f; uint32_t i; } cv = {(float)u};
            u = cv.i;
        }
        int32_t c = ((int32_t)log_e_q4[(u >> 23) & 255u] + log_m_q4[(u >> 16) & 127u] - off) >> 4;
        o[bin_of[k]] = c <= 0 ? 0 : c >= 255 ? 255 : (uint8_t)c;
        a[k] = 0;
    }
    st.emit_pos = end;
    if (end < spec_n) return;
    memcpy(frame_out, &st.emit_h, sizeof(spec_header_t));
    size_t len = sizeof(spec_header_t) + spec_n;
    uint32_t crc = esp_rom_crc32_le(0, frame_out, len);
    memcpy(frame_out + len, &crc, 4);
    ring_result_t *r = st.res;
    if (txq_push(frame_out, len + 4)) {
        st.last_ok = esp_timer_get_time();
        r->frames++;
        st.dropped = false;
    } else {
        r->drops++;
        st.dropped = true;
    }
    st.emitting = false;
#if CONFIG_IDF_TARGET_ESP32S3
    if (st.cfg->stats) stats_maybe(false);
#endif
}

/* Retire a bank before RF reuses it. Once unpacking is complete the FFT owns
 * its input copy, so it can finish after retirement. Defer frame closure until
 * that FFT commits; never mix a partial transform into a later frame. */
RING_HOT static void unit_done(void) {
    unsigned b = st.fifo[0];
    work_t *w = &st.work[b];
    if (w->next_block < w->blocks) {
        st.res->abandoned += (w->blocks - w->next_block + st.cfg->stride - 1) / st.cfg->stride;
        st.frame_flags |= 2;
    }
    if (st.phase == BLK_UNPACK && st.blk_bank == b) {
        st.phase = BLK_IDLE;
        st.res->abandoned++;
        st.frame_flags |= 2;
    }
    w->pending = false;
    for (unsigned i = 1; i < st.fifo_len; i++) st.fifo[i - 1] = st.fifo[i];
    st.fifo_len--;
    if (!st.frame_units) {
        st.frame_index = w->index;
        st.frame_gain = w->gain;
    }
    st.frame_pairs += w->count;
#if CONFIG_IDF_TARGET_ESP32S3
    sx.pairs_done += w->count;
#endif
    /* close only with at least one FFT (or after 4x upf units without one) */
    if (++st.frame_units >= st.cfg->units_per_frame && st.phase == BLK_IDLE &&
        (st.frame_ffts || st.frame_units >= 4u * st.cfg->units_per_frame))
        frame_close();
}

/* Bank b is about to be overwritten: drop whatever work it still holds. */
RING_HOT static void release_bank(unsigned b) {
#if CONFIG_IDF_TARGET_ESP32S3
    while (st.work[b].pending) unit_done(); /* FIFO order: older units retire first */
#endif
#if !CONFIG_IDF_TARGET_ESP32S3
    if(scalar.phase==3 && scalar.bank==b){scalar.phase=0;st.res->abandoned++;}
    st.work[b].pending=false;
#endif
    (void)b;
}

/* One work slice; returns false when there is nothing to do. */
RING_HOT static bool work_slice(void) {
    uint32_t t0 = esp_cpu_get_cycle_count();
    if (st.emitting) {
        emit_chunk();
    } else if (st.phase == BLK_FFT) {
        S3_FFT(fft_buf, spec_n);
        spec_remove_dc();
        st.phase = BLK_ACCUM;
        st.pos = 0;
    } else if (st.phase == BLK_ACCUM) {
        /* Commit a whole FFT atomically with respect to unit retirement. */
        unsigned end = spec_n;
        spec_accumulate(st.cfg->max_hold, st.pos, end);
        st.pos = end;
        if (end == spec_n) {
            st.phase = BLK_IDLE;
            st.frame_ffts++;
            st.res->ffts++;
            if (st.frame_units >= st.cfg->units_per_frame) frame_close();
        }
    } else if (st.phase == BLK_UNPACK) {
        unsigned end = st.pos + CHUNK < spec_n ? st.pos + CHUNK : spec_n;
        spec_unpack(bank_ptr(st.blk_bank), st.blk_at, st.pos, end);
        st.pos = end;
        if (end == spec_n) st.phase = BLK_FFT;
    } else {
        if (!st.fifo_len) return false;
        unsigned b = st.fifo[0];
        work_t *w = &st.work[b];
        if (w->next_block >= w->blocks) {
            unit_done();
        } else {
            st.blk_bank = b;
            st.blk_at = w->first + (w->next_block << spec_log2);
            w->next_block += st.cfg->stride;
            st.phase = BLK_UNPACK;
            st.pos = 0;
        }
    }
    uint32_t dt = esp_cpu_get_cycle_count() - t0;
    if (dt > st.res->work_max) st.res->work_max = dt;
    return true;
}


#if CONFIG_IDF_TARGET_ESP32S3
/* ---- core 1 SPEC worker ------------------------------------------------- */
/* Core 0 keeps the ring (poll, sentinels, bank switch, unit location) and the
 * USB output; core 1 does all spectral work: unpack -> FFT -> accumulate ->
 * frame coding -> txq. Handshake through internal SRAM (no data cache):
 *   - core 0 posts each located unit with a sequence number and marks its
 *     bank valid (bank_seq[b] = seq); core 1 clears it when done with it;
 *   - before refilling a bank with sentinels, core 0 revokes it
 *     (bank_seq[b] = 0) and waits while core 1 unpacks from it (busy == b+1);
 *   - core 1 sets busy, then re-checks bank_seq before every unpack (Dekker,
 *     memw on both sides), so it never reads a bank that is being refilled.
 * Core 1 runs only IRAM code on DRAM data, interrupts masked, no flash. */
#define C1_QUEUE 8u
#define MEMW() __asm__ volatile("memw" ::: "memory")
typedef struct {
    uint32_t seq, bank, first, count, blocks;
    uint32_t start; /* first block to transform (stride phase carried across units) */
    uint64_t index;
    uint8_t gain;
} c1_unit_t;
static volatile struct {
    uint32_t alive, run, posted, taken, end, done, busy, block_max;
    uint32_t bank_seq[RING_BANKS];
} c1;
static c1_unit_t c1q[C1_QUEUE];
static bool c1_ok, c1_enabled = true;
extern void s3_core1_entry(void);

bool ring_capture_dual_active(void) { return c1_ok && c1_enabled; }
bool ring_capture_assist = true;      /* core 0 helps core 1 (DUAL 2 = without) */
uint32_t ring_capture_c0_blocks;      /* blocks core 0 handed off in the last run */
bool ring_capture_core1_alive(void) { return c1_ok; }
void ring_capture_set_dual(bool on) { c1_enabled = on; }

IRAM_ATTR static bool c1_txq_push(const uint8_t *d, uint32_t n) {
    uint32_t head = txq_head, tail = *(volatile uint32_t *)&txq_tail;
    if (TXQ_SIZE - (head - tail) < n) return false;
    for (uint32_t i = 0; i < n; i++) txq[(head + i) & (TXQ_SIZE - 1u)] = d[i];
    MEMW();
    *(volatile uint32_t *)&txq_head = head + n;
    return true;
}

/* Frame being encoded on core 1 (source = the accumulator swapped out). */
static struct {
    uint32_t pending, k, crc, crc_at, len;
    int32_t off;
    bool max_hold;
    uint32_t *src;
} c1enc;

/* One slice of the pending frame: 256 bins, then 1 KB of CRC, then the push. */
IRAM_ATTR static void c1_encode_step(void) {
    if (!c1enc.pending) return;
    if (c1enc.k < spec_n) {
        unsigned end = c1enc.k + 256u;
        if (end > spec_n) end = spec_n;
        uint8_t *o = frame_out + sizeof(spec_header_t);
        uint32_t *a = c1enc.src;
        const int32_t off = c1enc.off;
        for (unsigned k = c1enc.k; k < end; k++) {
            uint32_t u = a[k];
            if (c1enc.max_hold) {
                union { float f; uint32_t i; } cv = {(float)u};
                u = cv.i;
            }
            int32_t c = ((int32_t)log_e_q4[(u >> 23) & 255u] + log_m_q4[(u >> 16) & 127u] - off) >> 4;
            o[bin_of[k]] = c <= 0 ? 0 : c >= 255 ? 255 : (uint8_t)c;
            a[k] = 0;
        }
        c1enc.k = end;
        return;
    }
    if (c1enc.crc_at < c1enc.len) {
        uint32_t n = c1enc.len - c1enc.crc_at;
        if (n > 1024u) n = 1024u;
        c1enc.crc = esp_rom_crc32_le(c1enc.crc, frame_out + c1enc.crc_at, n);
        c1enc.crc_at += n;
        return;
    }
    ring_result_t *r = st.res;
    uint32_t len = c1enc.len, crc = c1enc.crc;
    for (unsigned i = 0; i < 4; i++) frame_out[len + i] = (uint8_t)(crc >> (8u * i));
    if (c1_txq_push(frame_out, len + 4)) {
        r->frames++;
        st.dropped = false;
    } else {
        r->drops++;
        st.dropped = true;
    }
    c1enc.pending = 0;
    if (st.cfg->stats) stats_maybe(true);
}

IRAM_ATTR static void c1_emit_frame(void) {
    while (c1enc.pending) c1_encode_step(); /* frame_out holds the previous frame */
    ring_result_t *r = st.res;
    spec_header_t h;
    h.magic = SPEC_MAGIC;
    h.frame = r->frames + r->drops;
    h.pair_index = st.frame_index;
    h.pairs = st.frame_pairs;
    h.ffts = (uint16_t)st.frame_ffts;
    h.flags = (uint8_t)((st.cfg->max_hold ? 1 : 0) | st.frame_flags | (st.dropped ? 4 : 0));
    h.gain = st.frame_gain;
    h.drops = (uint16_t)(r->drops > 65535 ? 65535 : r->drops);
    h.nfft_log2 = (uint8_t)spec_log2;
    h.db_step = 2;
    int32_t off = 0;
    if (!st.cfg->max_hold && st.frame_ffts >= 2) { /* mean: subtract log2(ffts) */
        union { float f; uint32_t i; } cv = {(float)st.frame_ffts};
        off = (int32_t)log_e_q4[(cv.i >> 23) & 255u] + log_m_q4[(cv.i >> 16) & 127u] - 8;
    }
    const uint8_t *hp = (const uint8_t *)&h;
    for (unsigned i = 0; i < sizeof(spec_header_t); i++) frame_out[i] = hp[i];
    c1enc.src = (uint32_t *)accum;
    c1enc.off = off;
    c1enc.max_hold = st.cfg->max_hold;
    c1enc.k = c1enc.crc = c1enc.crc_at = 0;
    c1enc.len = sizeof(spec_header_t) + spec_n;
    c1enc.pending = 1;
    accum = accum == accum_buf[0] ? accum_buf[1] : accum_buf[0]; /* already zeroed */
    st.frame_units = st.frame_ffts = st.frame_pairs = 0;
    st.frame_flags = 0;
}

/* Unpack samples [from, to) of the block at ring index `at` into dst (same
 * layout as fft_buf). from/to multiples of 8. PIE everywhere; across the ring
 * wrap only the <= 8 straddling samples go through the scalar loop. */
IRAM_ATTR static void unpack_seg(int16_t *dst, const uint32_t *p, unsigned at, unsigned from, unsigned to) {
    unsigned first = (at + from) & RING_MASK, count = to - from;
    unsigned n1 = RING_PAIRS - first; /* samples before the wrap */
    if (n1 >= count) {
        s3_unpack_iq10_win(p + first, dst + 2 * from, win2 + 2 * from, count / 8u);
        return;
    }
    unsigned k8 = n1 & ~7u, r8 = (n1 + 7u) & ~7u;
    if (k8) s3_unpack_iq10_win(p + first, dst + 2 * from, win2 + 2 * from, k8 / 8u);
    for (unsigned i = from + k8; i < from + r8; i++) {
        uint32_t w = p[(at + i) & RING_MASK];
        int32_t I = (int32_t)(w << 22) >> 22, Q = (int32_t)(w << 12) >> 22;
        dst[2 * i] = (int16_t)((I * window_q15[i]) >> 9);
        dst[2 * i + 1] = (int16_t)((Q * window_q15[i]) >> 9);
    }
    if (count > r8)
        s3_unpack_iq10_win(p + (r8 - n1), dst + 2 * (from + r8), win2 + 2 * (from + r8), (count - r8) / 8u);
}

/* Buffer-parameter versions of the block stages (core 0 hand-off path). */
IRAM_ATTR static void unpack_to(int16_t *dst, const uint32_t *p, unsigned at) {
    unpack_seg(dst, p, at, 0, spec_n);
}
IRAM_ATTR static void remove_dc_buf(int16_t *x) { dc_track(x); }
IRAM_ATTR static void accumulate_buf(const int16_t *x, bool max_hold) {
    if (max_hold) {
        uint32_t *pk = (uint32_t *)accum;
        for (unsigned k = 0; k < spec_n; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            uint32_t power = (uint32_t)(re * re) + (uint32_t)(im * im);
            if (power > pk[k]) pk[k] = power;
        }
    } else {
        for (unsigned k = 0; k < spec_n; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            accum[k] += (float)((uint32_t)(re * re) + (uint32_t)(im * im));
        }
    }
}

/* Shared block cursor of the unit core 1 is working on, and the core-0
 * hand-off buffer (one FFT output waiting for core 1's accumulator). */
static volatile struct {
    uint32_t seq;        /* unit open for claims, 0 = none */
    uint32_t bank, first, blocks, stride;
    uint32_t claim;      /* next block index (atomic) */
    uint32_t c0_seq;     /* core 0 is between claim and hand-off of this unit */
    uint32_t hb_full;    /* hand-off buffer holds an FFT for core 1 */
    uint32_t hb_seq;
    uint32_t done;       /* blocks accumulated for the open unit */
    uint32_t c0_blocks;  /* blocks core 0 handed off (whole run) */
} cl;
static int16_t *hbuf; /* 8 KB, heap: BSS must end below the RF ring */

IRAM_ATTR static bool claim_block(uint32_t *out) {
    for (;;) {
        uint32_t b = cl.claim;
        if (b >= cl.blocks) return false;
        if (xt_utils_compare_and_set(&cl.claim, b, b + cl.stride)) {
            *out = b;
            return true;
        }
    }
}

IRAM_ATTR static void c1_take_handoff(void) {
    accumulate_buf(hbuf, st.cfg->max_hold);
    st.frame_ffts++;
    st.res->ffts++;
    cl.done++;
    MEMW();
    cl.hb_full = 0;
}

IRAM_ATTR static void c1_unit(const c1_unit_t *u) {
    const uint32_t tu = esp_cpu_get_cycle_count();
    const unsigned stride = st.cfg->stride;
    const unsigned total = u->start < u->blocks ? (u->blocks - u->start + stride - 1) / stride : 0;
    cl.bank = u->bank;
    cl.first = u->first;
    cl.blocks = u->blocks;
    cl.stride = stride;
    cl.claim = u->start;
    cl.done = 0;
    MEMW();
    cl.seq = u->seq; /* open for claims (core 0 too) */
    MEMW();
    bool revoked = false;
    for (;;) {
        if (cl.hb_full) c1_take_handoff();
        uint32_t blk;
        if (revoked || !claim_block(&blk)) break;
        uint32_t t0 = esp_cpu_get_cycle_count();
        const uint32_t *bp = bank_ptr(u->bank);
        unsigned at = u->first + (blk << spec_log2);
        for (unsigned s0 = 0; s0 < spec_n && !revoked; s0 += 256u) {
            c1.busy = u->bank + 1u;
            MEMW();
            if (c1.bank_seq[u->bank] != u->seq) revoked = true; /* core 0 needs the bank */
            else unpack_seg(fft_buf, bp, at, s0, s0 + 256u);
            MEMW();
            c1.busy = 0;
        }
        if (revoked) {
            cl.claim = cl.blocks; /* stop further claims; the partial block is dropped */
            continue;
        }
        S3_FFT(fft_buf, spec_n);
        spec_remove_dc();
        spec_accumulate(st.cfg->max_hold, 0, spec_n);
        st.frame_ffts++;
        st.res->ffts++;
        cl.done++;
        uint32_t dt = esp_cpu_get_cycle_count() - t0;
        if (dt > c1.block_max) c1.block_max = dt;
    }
    /* Close: no more claims; collect core 0's block still in flight. */
    cl.claim = cl.blocks;
    MEMW();
    while (cl.c0_seq == u->seq || (cl.hb_full && cl.hb_seq == u->seq)) {
        if (cl.hb_full) c1_take_handoff();
    }
    cl.seq = 0;
    MEMW();
    if (cl.done < total) {
        st.res->abandoned += total - cl.done;
        st.frame_flags |= 2;
    }
    if (c1.bank_seq[u->bank] == u->seq) c1.bank_seq[u->bank] = 0; /* bank free */
    MEMW();
    if (!st.frame_units) {
        st.frame_index = u->index;
        st.frame_gain = u->gain;
    }
    st.frame_pairs += u->count;
    sx.pairs_done += u->count;
    if (++st.frame_units >= st.cfg->units_per_frame && (st.frame_ffts || st.frame_units >= 4u * st.cfg->units_per_frame))
        c1_emit_frame();
    ld.c1_busy += esp_cpu_get_cycle_count() - tu;
}

/* Core 0: process one block of core 1's open unit into the hand-off buffer.
 * Called from the poll loop only when it fits before the next ring deadline. */
IRAM_ATTR static void c0_assist(void) {
    if (cl.hb_full) return;
    uint32_t seq = cl.seq;
    if (!seq) return;
    cl.c0_seq = seq;
    MEMW();
    uint32_t blk;
    if (cl.seq != seq || !claim_block(&blk)) {
        cl.c0_seq = 0;
        return;
    }
    unsigned bank = cl.bank;
    if (c1.bank_seq[bank] != seq) { /* revoked meanwhile (core 0 itself) */
        cl.c0_seq = 0;
        return;
    }
    unpack_to(hbuf, bank_ptr(bank), cl.first + (blk << spec_log2));
    S3_FFT(hbuf, spec_n);
    remove_dc_buf(hbuf);
    cl.hb_seq = seq;
    MEMW();
    cl.hb_full = 1;
    cl.c0_blocks++;
    MEMW();
    cl.c0_seq = 0;
}

IRAM_ATTR void s3_core1_main(void) {
    c1.alive = 1;
    MEMW();
    uint32_t run_seen = c1.run;
    for (;;) {
        while (c1.run == run_seen) {
        }
        run_seen = c1.run;
        MEMW();
        uint32_t taken = 0;
        for (;;) {
            MEMW();
            if (taken == c1.posted) {
                c1_encode_step();
                if (!c1.end) continue;
                MEMW();
                if (taken == c1.posted) break; /* end is written after the last post */
                continue;
            }
            c1_unit_t u = c1q[taken % C1_QUEUE];
            c1_unit(&u);
            taken++;
            c1.taken = taken;
            c1_encode_step();
        }
        if (st.frame_units) c1_emit_frame();
        while (c1enc.pending) c1_encode_step();
        MEMW();
        c1.done = 1;
    }
}

static inline void c1_revoke(unsigned b) {
    c1.bank_seq[b] = 0;
    MEMW();
    while (c1.busy == b + 1u) {
    }
}

static void c1_start(void) {
    hbuf = heap_caps_aligned_alloc(16, 2 * RING_SPEC_NFFT_MAX * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!hbuf) return; /* no second core: single-core SPEC */
    cpu_utility_ll_unstall_cpu(1);
    cpu_utility_ll_enable_clock_and_reset_app_cpu();
    ets_set_appcpu_boot_addr((uint32_t)s3_core1_entry);
    int64_t t0 = esp_timer_get_time();
    while (!c1.alive && esp_timer_get_time() - t0 < 50000) {
    }
    c1_ok = c1.alive != 0;
}
#endif /* CONFIG_IDF_TARGET_ESP32S3: core 1 */

RING_HOT static void fail(ring_result_t *r, ring_status_t code, uint32_t detail) {
    if (!r->status) {
        r->status = code;
        r->detail = detail;
    }
}

#if CONFIG_IDF_TARGET_ESP32C3
/* C3 exposes the live RF SRAM to the CPU. Copy a window safely behind the
 * writer; FFT work uses the copy and cannot be overwritten by RF. */
static void live_ring_run(const ring_config_t *cfg, ring_result_t *r) {
    uint32_t owner=REG_READ(DUMP_BANK_SELECT_REG);
    REG_WRITE(DUMP_CTRL_REG,0);REG_WRITE(DUMP_CONFIG_REG,DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_BANK_SELECT_REG,(owner&~7u)|2u|8u);
    REG_WRITE(DUMP_CTRL_REG,DUMP_CTRL_CIRCULAR|DUMP_CTRL_RUN);
    unsigned previous=REG_READ(DUMP_WRITE_INDEX_REG)&RING_MASK;
    uint64_t total=0;
    int64_t start=esp_timer_get_time(),last_poll=start,last_yield=start;st.last_ok=start;
    for(;;) {
        /* Only the sample copy is critical. Interrupts and the scheduler run
         * throughout the FFT, preserving both watchdog protections. */
        unsigned irq=portSET_INTERRUPT_MASK_FROM_ISR();
        int64_t now=esp_timer_get_time();
        unsigned current=REG_READ(DUMP_WRITE_INDEX_REG)&RING_MASK;
        uint64_t delta=(current-previous)&RING_MASK;
        uint64_t estimated=(uint64_t)(now-last_poll)*80u;
        if(estimated>delta+RING_PAIRS/2)delta+=((estimated-delta+RING_PAIRS/2)/RING_PAIRS)*RING_PAIRS;
        total+=delta;previous=current;last_poll=now;
        if(cfg->mode==RING_MODE_SPEC && !scalar.phase && total>spec_n+512) {
            unsigned before=esp_cpu_get_cycle_count();
            scalar_accept(0,(current-spec_n-512)&RING_MASK,total-spec_n-512);
            while(scalar.phase==3)scalar_work();
            unsigned copy_cycles=esp_cpu_get_cycle_count()-before;
            if(copy_cycles>(RING_PAIRS-spec_n-512)*cycles_per_pair(0)){
                scalar.phase=0;fail(r,RING_FAIL_AGE,copy_cycles);
            }
            r->units++;
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
        if(r->status || (cfg->duration_ms && now-start >= (int64_t)cfg->duration_ms*1000))break;
        if(burst_serial_stop_requested()){r->stopped_by_host=true;break;}
        if(!cfg->duration_ms && now-st.last_ok>2000000){r->stopped_by_host=true;break;}
        scalar_work();txq_pump();
        if(now-last_yield>=10000){vTaskDelay(1);last_yield=now;}
    }
    REG_WRITE(DUMP_CTRL_REG,0);REG_WRITE(DUMP_BANK_SELECT_REG,owner);
    r->elapsed_us=esp_timer_get_time()-start;r->pairs=total;
    while(scalar_work())txq_pump();
    int64_t deadline=esp_timer_get_time()+500000;
    while(txq_head!=txq_tail && esp_timer_get_time()<deadline){txq_pump();vTaskDelay(1);}
}
#endif


/* Generic run-loop hooks for the S3 second core (no-ops elsewhere). */
#if CONFIG_IDF_TARGET_ESP32S3
#define LD_C0(dt) (ld.c0_busy += (dt))
static inline void reclaim_bank(bool dual, unsigned b) { if (dual) c1_revoke(b); else release_bank(b); }
static inline bool bank_idle(bool dual, unsigned b) { return dual ? c1.bank_seq[b] == 0 : !st.work[b].pending; }
#else
#define LD_C0(dt) ((void)(dt))
static inline void reclaim_bank(bool dual, unsigned b) { (void)dual; release_bank(b); }
static inline bool bank_idle(bool dual, unsigned b) { (void)dual; return !st.work[b].pending; }
#endif

RING_HOT void ring_capture_run(const ring_config_t *cfg, ring_result_t *r) {
    memset(r, 0, sizeof(*r));
    memset(&st, 0, sizeof(st));
#if !CONFIG_IDF_TARGET_ESP32S3
    memset(&scalar,0,sizeof(scalar));
#endif
    st.cfg = cfg;
    st.res = r;
    txq_head = txq_tail = 0;
    const bool spec = cfg->mode == RING_MODE_SPEC;
    const bool capture = cfg->mode == RING_MODE_CAPTURE;
    if ((!ring_capture_rate_hz(cfg->rate)) ||
        (capture && (cfg->capture_units < 1 || cfg->capture_units > RING_BANKS)) ||
        (spec && (!dsp_ready || !ring_capture_valid_nfft(cfg->nfft) || !cfg->stride || !cfg->units_per_frame))) {
        fail(r, RING_FAIL_ARG, 0);
        return;
    }
    if (spec) spec_setup(cfg->nfft);
    memset(accum_buf, 0, sizeof(accum_buf));
    accum = accum_buf[0];
#if CONFIG_IDF_TARGET_ESP32S3
    c1enc.pending = 0;
#endif

    #if CONFIG_IDF_TARGET_ESP32C3
    live_ring_run(cfg,r);return;
    #endif
    const unsigned cpp = cycles_per_pair(cfg->rate);
    const uint32_t max_age = (RING_PAIRS - 128u) * cpp;
    const uint32_t settle = 16u * cpp;
    const uint32_t ctrl = DUMP_CTRL_CIRCULAR | rate_bits(cfg->rate);
    const int64_t duration_us = (int64_t)cfg->duration_ms * 1000;
    const uint32_t bank_sel_saved = REG_READ(DUMP_BANK_SELECT_REG);
    uint32_t start_probe[RING_BANKS] = {0}, end_probe[RING_BANKS] = {0};

    for (unsigned b = 0; b < RING_BANKS; b++) fill_sentinels(b, 0, RING_PAIRS);

#if CONFIG_IDF_TARGET_ESP32S3
    if (spec) { /* warm caches/tables; bank 2 holds sentinels only */
        spec_unpack(bank_ptr(2), 0, 0, spec_n);
        uint32_t t_fft = esp_cpu_get_cycle_count();
        S3_FFT(fft_buf, spec_n);
        t_fft = esp_cpu_get_cycle_count() - t_fft;
        spec_accumulate(false, 0, spec_n);
        memset(accum_buf, 0, sizeof(accum_buf));
        accum = accum_buf[0];
        c1enc.pending = 0;
        /* Seed the slice budget with the real FFT time of this size, so the
         * first slices are gated correctly (+15 % for cache/bus variation). */
        r->work_max = t_fft + t_fft / 7u;
    }
    const bool dual = spec && ring_capture_dual_active();
    memset(&sx, 0, sizeof(sx));
    ld.c0_busy = ld.c1_busy = 0;
    sx.heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    sx.heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    sx.mode = (uint16_t)((dual ? 1 : 0) | (dual && ring_capture_assist ? 2 : 0));
    uint32_t c1_seq = 0, c0_lost = 0, tail_seen = 0;
    uint32_t c0_cost = r->work_max + r->work_max / 3u; /* warm-up FFT + margin; grows to the max seen */
    if (dual) { /* hand the run to core 1 (idle since its last run) */
        c1.posted = c1.taken = c1.end = c1.done = c1.busy = c1.block_max = 0;
        cl.seq = cl.c0_seq = cl.hb_full = cl.c0_blocks = 0;
        for (unsigned i = 0; i < RING_BANKS; i++) c1.bank_seq[i] = 0;
        MEMW();
        c1.run = c1.run + 1u;
    }
#else
    const bool dual = false;
#endif
    uint32_t stride_phase = 0; /* first block of the next unit on the stride grid */
    dc_ok = 0;
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
#if CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32C6
#if CONFIG_IDF_TARGET_ESP32C6
    REG_WRITE(0x600a9804,0xffffffffu);REG_WRITE(0x600a9814,0x7ffffu);REG_WRITE(0x600a980c,0xffffffffu);
    REG_WRITE(DUMP_WRITE_INDEX_REG,(REG_READ(DUMP_WRITE_INDEX_REG)&~0x00078000u)|(15u<<15));
#else
    REG_WRITE(0x600a900cu,0);
    REG_WRITE(0x600a9c04u,0xffffffffu);
    REG_SET_BIT(0x600a0800u,4);
    REG_CLR_BIT(0x600a20b4u,1);
    REG_WRITE(DUMP_WRITE_INDEX_REG,(REG_READ(DUMP_WRITE_INDEX_REG)&~0x00fe0000u)|(15u<<17)|(cfg->rate<<21));
#endif
    REG_WRITE(DUMP_CTRL_REG, ctrl|(1u<<18));
#endif
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    select_banks(bank_sel_saved,1u);
    int64_t t_start = esp_timer_get_time();
    st.last_ok = t_start;
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_RUN);
    uint32_t epoch = esp_cpu_get_cycle_count();
    const uint32_t w0 = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;

    /* Unit 0 starts near w0; its exact start is found after the switch. */
    uint32_t expected = (w0 - 256u) & RING_MASK; /* poll reference for unit 0 */
    uint64_t index = 0;
    unsigned b = 0;
    bool prepared = false;

    for (unsigned unit = 0;; unit++) {
        const unsigned next = (b + 1) % RING_BANKS;
        const bool need_next = !capture || unit + 1 < cfg->capture_units;

        /* 1. Poll, doing work slices and output between polls. The stop
         *    decision is made mid-unit so nothing but the index read sits
         *    between the threshold and the bank write. */
        uint32_t write_index, written;
        bool stop = !need_next, stop_checked = false;
        for (;;) {
            write_index = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;
            written = (write_index - expected) & RING_MASK;
            uint32_t age = esp_cpu_get_cycle_count() - epoch;
            if (age > max_age) { fail(r, RING_FAIL_AGE, age); break; }
            if (written >= THRESHOLD) {
                if (need_next && !prepared) { /* a long slice ran past the deadline */
                    uint32_t tp = esp_cpu_get_cycle_count();
                    reclaim_bank(dual, next);
                    start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                    end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                    fill_sentinels(next, start_probe[next], START_GUARD);
                    fill_sentinels(next, end_probe[next], END_GUARD);
                    prepared = true;
                    LD_C0(esp_cpu_get_cycle_count() - tp);
                    write_index = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;
                    written = (write_index - expected) & RING_MASK;
                }
                break;
            }
            if (!stop_checked && written >= THRESHOLD / 2) {
                stop_checked = true;
                if (!stop && duration_us && esp_timer_get_time() - t_start >= duration_us) stop = true;
                /* Nobody has read a frame for 2 s (host closed or hung): end
                 * an open-ended SPEC run instead of streaming forever. */
#if CONFIG_IDF_TARGET_ESP32S3
                if (dual && txq_tail != tail_seen) { /* core 0 sees the host reading */
                    tail_seen = txq_tail;
                    st.last_ok = esp_timer_get_time();
                }
#endif
                if (!stop && spec && !duration_us && esp_timer_get_time() - st.last_ok > 2000000) {
                    stop = true;
                    r->stopped_by_host = true;
                }
                if (!stop && !capture && ring_input_available()) {
                    stop = true;
                    r->stopped_by_host = true;
                }
                continue;
            }
            /* Prepare the next bank as soon as its old unit is retired, and
             * unconditionally with one late-limit of margin left. */
            /* The forced path may retire a unit and emit a frame (~20 k
             * cycles), so its deadline scales with the sample rate. */
#if CONFIG_IDF_TARGET_ESP32S3
            /* dual: core 1 does the blocks, core 0 only prepares banks and pumps */
            const unsigned prep_cycles = !dual && r->work_max > 20000u ? r->work_max : 20000u;
#else
            const unsigned prep_cycles=12000;
#endif
            uint32_t prep_pairs = prep_cycles / cpp + 1024u;
            if (need_next && !prepared &&
                (bank_idle(dual, next) ||
                 written + prep_pairs + LATE_LIMIT >= THRESHOLD)) {
                uint32_t tp = esp_cpu_get_cycle_count();
                reclaim_bank(dual, next);
                start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                fill_sentinels(next, start_probe[next], START_GUARD);
                fill_sentinels(next, end_probe[next], END_GUARD);
                prepared = true;
                LD_C0(esp_cpu_get_cycle_count() - tp);
                continue;
            }
            if (written + 2048u < THRESHOLD && *(volatile uint32_t *)&txq_head != txq_tail) {
#if CONFIG_IDF_TARGET_ESP32S3
                uint32_t tq = esp_cpu_get_cycle_count();
                txq_pump();
                ld.c0_busy += esp_cpu_get_cycle_count() - tq;
#else
                txq_pump();
#endif
            }
            /* Start a slice only if the longest one seen so far still fits
             * before the switch; at 40/80 Msps most blocks are skipped. */
#if CONFIG_IDF_TARGET_ESP32S3
            if (dual && ring_capture_assist) {
                /* one core-0 block (unpack + FFT) must end before the bank
                 * preparation deadline, or before the switch if prepared */
                uint32_t c0_pairs = c0_cost / cpp + 256u;
                uint32_t limit = prepared || !need_next ? THRESHOLD + LATE_LIMIT / 2
                                                        : THRESHOLD - prep_pairs - LATE_LIMIT;
                if (written + c0_pairs < limit) {
                    uint32_t t0 = esp_cpu_get_cycle_count();
                    c0_assist();
                    uint32_t dt = esp_cpu_get_cycle_count() - t0;
                    if (dt > c0_cost) c0_cost = dt;
                    ld.c0_busy += dt;
                }
            }
#endif
            if (spec && !dual) {
#if CONFIG_IDF_TARGET_ESP32S3
                const unsigned minimum_slice=20000;
#else
                const unsigned minimum_slice=2000;
#endif
                uint32_t slice_cycles = r->work_max > minimum_slice ? r->work_max : minimum_slice;
                uint32_t slice_pairs = slice_cycles / cpp + 128u;
#if CONFIG_IDF_TARGET_ESP32S3
                if (written + slice_pairs + 512u < THRESHOLD + LATE_LIMIT) {
                    uint32_t tw = esp_cpu_get_cycle_count();
                    if (work_slice()) ld.c0_busy += esp_cpu_get_cycle_count() - tw;
                }
#else
                if (written + slice_pairs + 1024u < THRESHOLD) scalar_work();
#endif
            }
        }
        if (r->status) break;

        uint32_t late = written - THRESHOLD;
        if (late > r->late_max) r->late_max = late;
        if (late > LATE_LIMIT) { fail(r, RING_FAIL_LATE, written); break; }

        /* 2. Stop or switch. */
        const bool last = stop;
        if (!last && !prepared) { fail(r, RING_FAIL_ARG, 1); break; } /* cannot happen */
        if (last) {
            REG_WRITE(DUMP_CTRL_REG, ctrl);
            select_banks(bank_sel_saved,0u);
        } else {
            select_banks(bank_sel_saved,1u << next);
            epoch = esp_cpu_get_cycle_count();
        }
        uint32_t t = esp_cpu_get_cycle_count();
        while (esp_cpu_get_cycle_count() - t < settle) {}

        /* 3. Locate the finished unit exactly. */
        uint32_t first, end;
        if (unit == 0) {
            first = find_first(b, (w0 - 1024u) & RING_MASK, 3072u);
            end = find_end(b, write_index, write_index, UNIT0_END_GUARD);
        } else {
            first = find_first(b, start_probe[b], START_GUARD);
            end = find_end(b, end_probe[b], write_index, END_GUARD);
        }
        if (first == NOT_FOUND || (unit && first != expected)) {
            fail(r, RING_FAIL_START, (first & 0xffffu) | (expected << 16));
            break;
        }
        if (end == NOT_FOUND) { fail(r, RING_FAIL_END, write_index); break; }
        uint32_t count = (end - first) & RING_MASK;
        if (count < (unit ? MIN_PAIRS : MIN_PAIRS - 1024u) || count > MAX_PAIRS) {
            fail(r, RING_FAIL_LENGTH, count);
            break;
        }

        if (capture) r->cap[unit] = (ring_unit_t){.bank = (uint16_t)b, .first = (uint16_t)first, .count = count};
        uint32_t start = 0, todo = 0;
        if (spec) {
            uint32_t nblk = count >> spec_log2;
            start = stride_phase;
            todo = start < nblk ? (nblk - start + cfg->stride - 1) / cfg->stride : 0;
            stride_phase = start + todo * cfg->stride - nblk;
        }
#if !CONFIG_IDF_TARGET_ESP32S3
        if (spec) {
            scalar_accept(b,first,index);
#else
        if (dual) {
            uint32_t i = c1.posted;
            if (i - c1.taken >= C1_QUEUE) { /* core 1 far behind: skip this unit */
                c0_lost += todo;
            } else {
                c1q[i % C1_QUEUE] = (c1_unit_t){.seq = ++c1_seq, .bank = b, .first = first, .count = count,
                                                .blocks = count >> spec_log2, .start = start, .index = index,
                                                .gain = (uint8_t)(bank_ptr(b)[first] >> 20)};
                c1.bank_seq[b] = c1_seq;
                MEMW();
                c1.posted = i + 1u;
            }
        } else if (spec) {
            work_t *w = &st.work[b];
            *w = (work_t){.pending = true, .first = first, .count = count, .next_block = start,
                          .blocks = count >> spec_log2, .index = index,
                          .gain = (uint8_t)(bank_ptr(b)[first] >> 20)};
            st.fifo[st.fifo_len++] = b;
#endif
        }
        r->units++;
        index += count;
        expected = end;
        b = next;
        prepared = false;
        if (last) break;
    }

    /* Writer stopped (normally or by failure). Finish queued work. */
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, bank_sel_saved);
    r->elapsed_us = (uint64_t)(esp_timer_get_time() - t_start);
    r->pairs = index;
    (void)host_input(); /* consume the stop request so the parser never sees it */
#if !CONFIG_IDF_TARGET_ESP32S3
    if (spec) {
        while(scalar_work())txq_pump();
#else
    if (dual) {
        MEMW();
        c1.end = 1;
        MEMW();
        while (!c1.done) txq_pump();
        r->abandoned += c0_lost;
        r->work_max = c1.block_max; /* longest core-1 block (unpack..accumulate) */
        ring_capture_c0_blocks = cl.c0_blocks;
    } else if (spec) {
        while (work_slice()) txq_pump();
        if (st.frame_units) {
            frame_close();
            while (st.emitting) emit_chunk();
        }
#endif
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);

    /* Drain the queue with a deadline; the host may have stopped reading. */
    int64_t deadline = esp_timer_get_time() + 500000;
    while (txq_head != txq_tail && esp_timer_get_time() < deadline) txq_pump();
}
