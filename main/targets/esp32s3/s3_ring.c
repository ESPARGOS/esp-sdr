/*
 * ESP32-S3 continuous capture ring.
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
 * Interrupts stay disabled for the whole run: no FreeRTOS tick, no Wi-Fi
 * ISR, no driver. Output goes straight into the USB Serial/JTAG FIFO from a
 * RAM queue; a full queue drops whole frames (counted, never blocks).
 * Requires CONFIG_ESP_INT_WDT=n.
 */
#include "s3_ring.h"

#include <math.h>
#include <string.h>

#include "dsps_fft2r.h"
/* esp-dsp aes3 FFT with the sum-branch bias corrected (s3_fft_rnd.S). */
extern int16_t *dsps_fft_w_table_sc16;
int s3_fft2r_sc16_rnd(int16_t *data, int N, int16_t *w);
#define S3_FFT(buf, n) s3_fft2r_sc16_rnd((buf), (int)(n), dsps_fft_w_table_sc16)
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hal/usb_serial_jtag_ll.h"
#include "soc/soc.h"

/* Placement of the ring control code (flash by default; see notes). */
#ifndef RING_HOT
#define RING_HOT
#endif
#define RING_MASK (S3_RING_PAIRS - 1u)
#define THRESHOLD S3_RING_THRESHOLD
#define SENTINEL 0xA5C33C5Au
#define START_GUARD 3072u  /* sentinels from a unit's earliest possible start */
#define END_GUARD 4096u    /* sentinels from a unit's earliest possible end */
#define LATE_LIMIT 2000u   /* switch lateness allowed: 2*LATE_LIMIT < END_GUARD */
#define UNIT0_END_GUARD 1024u /* unit 0: whole bank is sentinel, stay clear of its start */
#define MIN_PAIRS THRESHOLD
#define MAX_PAIRS (THRESHOLD + LATE_LIMIT + 64u)
#define NOT_FOUND 0xffffffffu

#define DUMP_CTRL_REG 0x60033d5cu
#define DUMP_WRITE_INDEX_REG 0x60033d60u
#define DUMP_CONFIG_REG 0x60033d90u
#define DUMP_BANK_SELECT_REG 0x600c101cu
#define DUMP_CTRL_RUN 0x80000000u
#define DUMP_CTRL_CIRCULAR 0x00024000u /* circular 16384-pair ring, IQ source 0 */
#define DUMP_CONFIG_IQ 0x000c2040u

_Static_assert(S3_RING_BANK_BASE + S3_RING_BANKS * 0x10000u == S3_RING_BANK_END, "bank map");
_Static_assert(START_GUARD + THRESHOLD <= S3_RING_PAIRS, "start window overlaps");
_Static_assert(END_GUARD + THRESHOLD <= S3_RING_PAIRS, "end window overlaps unit start");
_Static_assert(2u * LATE_LIMIT + 64u < END_GUARD && LATE_LIMIT + 64u < START_GUARD, "guards");

static inline uint32_t *bank_ptr(unsigned b) {
    return (uint32_t *)(S3_RING_BANK_BASE + b * 0x10000u);
}
const uint32_t *s3_ring_bank(unsigned b) { return bank_ptr(b); }

unsigned s3_ring_rate_hz(unsigned rate) {
    return rate == 6 ? 16000000u : rate == 1 ? 40000000u : 80000000u;
}
static uint32_t rate_bits(unsigned rate) {
    return rate == 6 ? (1u << 16) : rate == 1 ? (1u << 15) : 0;
}
static unsigned cycles_per_pair(unsigned rate) { /* 240 MHz CPU */
    return rate == 6 ? 15u : rate == 1 ? 6u : 3u;
}

/* ---- sentinel windows ------------------------------------------------- */

/* Fills ring indices [at, at+count) of bank b with sentinels, wrapping.
 * 128-bit PIE stores (from eSpDR capture.c, 0BSD): at 80 Msps the scalar
 * loop was too slow to prepare 7 k words before the next switch. */
RING_HOT static void fill_sentinels(unsigned b, unsigned at, unsigned count) {
    at &= RING_MASK;
    while (count) {
        unsigned n = S3_RING_PAIRS - at;
        if (n > count) n = count;
        uint32_t *p = bank_ptr(b) + at;
        unsigned head = (-at) & 3u;
        if (head > n) head = n;
        for (unsigned i = 0; i < head; i++) *p++ = SENTINEL;
        unsigned vectors = (n - head) / 4;
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
        for (unsigned i = head + vectors * 4; i < n; i++) *p++ = SENTINEL;
        count -= n;
        at = 0;
    }
    __asm__ volatile("memw" ::: "memory");
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

#define TXQ_SIZE 16384u /* 7 frames of 2048 bins, 56 of 256 */
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
    uint32_t used = txq_head - txq_tail;
    if (!used || !usb_serial_jtag_ll_txfifo_writable()) return;
    uint32_t off = txq_tail % TXQ_SIZE, n = TXQ_SIZE - off;
    if (n > used) n = used;
    if (n > 64) n = 64;
    int written = usb_serial_jtag_ll_write_txfifo(txq + off, n);
    usb_serial_jtag_ll_txfifo_flush();
    if (written > 0) txq_tail += (uint32_t)written;
}

RING_HOT static bool host_input(void) {
    if (!usb_serial_jtag_ll_rxfifo_data_available()) return false;
    uint8_t b;
    for (unsigned k = 0; k < 64 && usb_serial_jtag_ll_rxfifo_data_available(); k++) {
        if (usb_serial_jtag_ll_read_rxfifo(&b, 1) == 1 && b == '\n') break;
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

static int16_t window_q15[S3_SPEC_NFFT_MAX];
static int16_t fft_buf[2 * S3_SPEC_NFFT_MAX] __attribute__((aligned(16)));
static float accum[S3_SPEC_NFFT_MAX];     /* mean: power sum; max-hold: peak power */
static uint16_t bin_of[S3_SPEC_NFFT_MAX]; /* FFT output slot -> natural-order bin */
static uint8_t frame_out[sizeof(spec_header_t) + S3_SPEC_NFFT_MAX + 4];
static unsigned spec_n, spec_log2;        /* FFT size of the current run */

static bool dsp_ready;
void s3_ring_init(void) {
    if (dsp_ready) return;
    dsp_ready = dsps_fft2r_init_sc16(NULL, S3_SPEC_NFFT_MAX) == ESP_OK;
}

bool s3_ring_valid_nfft(unsigned n) { return n == 256 || n == 1024 || n == 2048; }

/* Hann window and bit-reversal bin map for FFT size n; outside the timed run.
 * Scaling is independent of n: IQ10 << 6 in, the int16 FFT divides by n. */
static void spec_setup(unsigned n) {
    if (n == spec_n) return;
    unsigned log2n = 0;
    while ((1u << log2n) < n) log2n++;
    for (unsigned i = 0; i < n; i++) {
        window_q15[i] = (int16_t)lrintf(32767.0f * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / n)));
        unsigned r = 0;
        for (unsigned bit = 0; bit < log2n; bit++) r |= ((i >> bit) & 1u) << (log2n - 1 - bit);
        bin_of[i] = (uint16_t)r;
    }
    spec_n = n;
    spec_log2 = log2n;
}

/* log2 with ~0.005 absolute error (0.015 dB). */
static inline float fast_log2(float x) {
    union { float f; uint32_t i; } u = {x};
    float e = (float)(int)((u.i >> 23) & 255u) - 128.0f;
    u.i = (u.i & 0x007fffffu) | 0x3f800000u;
    return e + ((-1.0f / 3.0f) * u.f + 2.0f) * u.f - 2.0f / 3.0f;
}

/* Block pipeline, one slice per call so the ring is polled in between:
 * unpack (CHUNK samples, needs the bank) -> FFT (whole block) ->
 * accumulate (CHUNK bins). Only unpacking reads the capture bank. */
IRAM_ATTR static void spec_unpack(const uint32_t *p, unsigned at, unsigned from, unsigned to) {
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
IRAM_ATTR static void spec_remove_dc(void) {
    int32_t r0 = fft_buf[0], i0 = fft_buf[1];
    unsigned s1 = spec_n / 2, sm = spec_n - 1;
    fft_buf[2 * s1] = sat16(fft_buf[2 * s1] + r0 / 2);
    fft_buf[2 * s1 + 1] = sat16(fft_buf[2 * s1 + 1] + i0 / 2);
    fft_buf[2 * sm] = sat16(fft_buf[2 * sm] + r0 / 2);
    fft_buf[2 * sm + 1] = sat16(fft_buf[2 * sm + 1] + i0 / 2);
    fft_buf[0] = fft_buf[1] = 0;
}

IRAM_ATTR static void spec_accumulate(bool max_hold, unsigned from, unsigned to) {
    if (max_hold) {
        for (unsigned k = from; k < to; k++) {
            int32_t re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
            float power = (float)(re * re + im * im);
            unsigned bin = bin_of[k];
            if (power > accum[bin]) accum[bin] = power;
        }
    } else {
        for (unsigned k = from; k < to; k++) {
            int32_t re = fft_buf[2 * k], im = fft_buf[2 * k + 1];
            accum[bin_of[k]] += (float)(re * re + im * im);
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
    work_t work[S3_RING_BANKS];
    unsigned fifo[S3_RING_BANKS], fifo_len; /* banks with pending work, oldest first */
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
    float emit_scale;
    spec_header_t emit_h;
} st;

/* Close the accumulating frame; its bins are encoded by emit_chunk(). */
RING_HOT static void emit_chunk(void);
RING_HOT static void frame_close(void) {
    while (st.emitting) emit_chunk(); /* rare: previous frame not yet out */
    ring_result_t *r = st.res;
    st.emit_h = (spec_header_t){
        .magic = SPEC_MAGIC, .frame = r->frames + r->drops, .pair_index = st.frame_index,
        .pairs = st.frame_pairs, .ffts = (uint16_t)st.frame_ffts,
        .flags = (uint8_t)((st.cfg->max_hold ? 1 : 0) | st.frame_flags | (st.dropped ? 4 : 0)),
        .gain = st.frame_gain, .drops = (uint16_t)(r->drops > 65535 ? 65535 : r->drops),
        .nfft_log2 = (uint8_t)spec_log2, .db_step = 2,
    };
    st.emit_scale = st.cfg->max_hold ? 1.0f : 1.0f / (st.frame_ffts ? (float)st.frame_ffts : 1.0f);
    st.emit_pos = 0;
    st.emitting = true;
    st.frame_units = st.frame_ffts = st.frame_pairs = 0;
    st.frame_flags = 0;
}

RING_HOT static void emit_chunk(void) {
    uint8_t *o = frame_out + sizeof(spec_header_t);
    unsigned end = st.emit_pos + CHUNK < spec_n ? st.emit_pos + CHUNK : spec_n;
    const float scale = st.emit_scale;
    for (unsigned k = st.emit_pos; k < end; k++) {
        float v = accum[k] * scale;
        float code = v > 1.0f ? 2.0f * 3.0103f * fast_log2(v) + 0.5f : 0.0f;
        o[k] = code >= 255.0f ? 255 : (uint8_t)code;
        accum[k] = 0;
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
}

/* Retire the oldest pending unit (fully processed or abandoned). A block
 * still being unpacked from it is dropped; one already unpacked finishes. */
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
    if (++st.frame_units >= st.cfg->units_per_frame) frame_close();
}

/* Bank b is about to be overwritten: drop whatever work it still holds. */
RING_HOT static void release_bank(unsigned b) {
    while (st.work[b].pending) unit_done(); /* FIFO order: older units retire first */
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
        unsigned end = st.pos + CHUNK < spec_n ? st.pos + CHUNK : spec_n;
        spec_accumulate(st.cfg->max_hold, st.pos, end);
        st.pos = end;
        if (end == spec_n) {
            st.phase = BLK_IDLE;
            st.frame_ffts++;
            st.res->ffts++;
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

RING_HOT static void fail(ring_result_t *r, ring_status_t code, uint32_t detail) {
    if (!r->status) {
        r->status = code;
        r->detail = detail;
    }
}

RING_HOT void s3_ring_run(const ring_config_t *cfg, ring_result_t *r) {
    memset(r, 0, sizeof(*r));
    memset(&st, 0, sizeof(st));
    st.cfg = cfg;
    st.res = r;
    txq_head = txq_tail = 0;
    const bool spec = cfg->mode == RING_MODE_SPEC;
    const bool capture = cfg->mode == RING_MODE_CAPTURE;
    if ((cfg->rate != 0 && cfg->rate != 1 && cfg->rate != 6) ||
        (capture && (cfg->capture_units < 1 || cfg->capture_units > S3_RING_BANKS)) ||
        (spec && (!dsp_ready || !s3_ring_valid_nfft(cfg->nfft) || !cfg->stride || !cfg->units_per_frame))) {
        fail(r, RING_FAIL_ARG, 0);
        return;
    }
    if (spec) spec_setup(cfg->nfft);
    memset(accum, 0, sizeof(accum));

    const unsigned cpp = cycles_per_pair(cfg->rate);
    const uint32_t max_age = (S3_RING_PAIRS - 128u) * cpp;
    const uint32_t settle = 16u * cpp;
    const uint32_t ctrl = DUMP_CTRL_CIRCULAR | rate_bits(cfg->rate);
    const int64_t duration_us = (int64_t)cfg->duration_ms * 1000;
    const uint32_t bank_sel_saved = REG_READ(DUMP_BANK_SELECT_REG);
    uint32_t start_probe[S3_RING_BANKS] = {0}, end_probe[S3_RING_BANKS] = {0};

    for (unsigned b = 0; b < S3_RING_BANKS; b++) fill_sentinels(b, 0, S3_RING_PAIRS);

    if (spec) { /* warm caches/tables; bank 2 holds sentinels only */
        spec_unpack(bank_ptr(2), 0, 0, spec_n);
        uint32_t t_fft = esp_cpu_get_cycle_count();
        S3_FFT(fft_buf, spec_n);
        t_fft = esp_cpu_get_cycle_count() - t_fft;
        spec_accumulate(false, 0, spec_n);
        memset(accum, 0, sizeof(accum));
        /* Seed the slice budget with the real FFT time of this size, so the
         * first slices are gated correctly (+15 % for cache/bus variation). */
        r->work_max = t_fft + t_fft / 7u;
    }
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    REG_WRITE(DUMP_BANK_SELECT_REG, (bank_sel_saved & ~15u) | 1u);
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
        const unsigned next = (b + 1) % S3_RING_BANKS;
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
                    release_bank(next);
                    start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                    end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                    fill_sentinels(next, start_probe[next], START_GUARD);
                    fill_sentinels(next, end_probe[next], END_GUARD);
                    prepared = true;
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
                if (!stop && spec && !duration_us && esp_timer_get_time() - st.last_ok > 2000000) {
                    stop = true;
                    r->stopped_by_host = true;
                }
                if (!stop && !capture && usb_serial_jtag_ll_rxfifo_data_available()) {
                    stop = true;
                    r->stopped_by_host = true;
                }
                continue;
            }
            /* Prepare the next bank as soon as its old unit is retired, and
             * unconditionally with one late-limit of margin left. */
            /* The forced path may retire a unit and emit a frame (~20 k
             * cycles), so its deadline scales with the sample rate. */
            uint32_t prep_pairs = (r->work_max > 20000u ? r->work_max : 20000u) / cpp + 1024u;
            if (need_next && !prepared &&
                (!st.work[next].pending || written + prep_pairs + LATE_LIMIT >= THRESHOLD)) {
                release_bank(next);
                start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                fill_sentinels(next, start_probe[next], START_GUARD);
                fill_sentinels(next, end_probe[next], END_GUARD);
                prepared = true;
                continue;
            }
            txq_pump();
            /* Start a slice only if the longest one seen so far still fits
             * before the switch; at 40/80 Msps most blocks are skipped. */
            if (spec) {
                uint32_t slice_cycles = r->work_max > 20000u ? r->work_max : 20000u;
                uint32_t slice_pairs = slice_cycles / cpp + 128u;
                if (written + slice_pairs + 512u < THRESHOLD + LATE_LIMIT) work_slice();
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
            REG_WRITE(DUMP_BANK_SELECT_REG, bank_sel_saved & ~15u);
        } else {
            REG_WRITE(DUMP_BANK_SELECT_REG, (bank_sel_saved & ~15u) | (1u << next));
            epoch = esp_cpu_get_cycle_count();
        }
        uint32_t t = esp_cpu_get_cycle_count();
        while (esp_cpu_get_cycle_count() - t < settle) {}

        /* 3. Locate the finished unit exactly. */
        uint32_t first, end;
        if (unit == 0) {
            first = find_first(b, (w0 - 1024u) & RING_MASK, START_GUARD);
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
        if (spec) {
            work_t *w = &st.work[b];
            *w = (work_t){.pending = true, .first = first, .count = count, .next_block = 0,
                          .blocks = count >> spec_log2, .index = index,
                          .gain = (uint8_t)(bank_ptr(b)[first] >> 20)};
            st.fifo[st.fifo_len++] = b;
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
    if (spec) {
        while (work_slice()) txq_pump();
        if (st.frame_units) {
            frame_close();
            while (st.emitting) emit_chunk();
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);

    /* Drain the queue with a deadline; the host may have stopped reading. */
    int64_t deadline = esp_timer_get_time() + 500000;
    while (txq_head != txq_tail && esp_timer_get_time() < deadline) txq_pump();
}
