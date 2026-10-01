# ESP32-S3 Turbo Mode: dual-core SPEC and benchmarks

This note documents the second round of Turbo Mode (SPEC) work on the
ESP32-S3: where the CPU time went, what was changed, and what the changes
bought. All numbers were measured on an ESP32-S3 board over native USB
(Serial/JTAG) with the s3_prof profiling build and a Signal Hound VSG60 as a
CW reference source. Baseline is commit 83090c0.

Turbo Mode developed by Zoltan Doczi from [https://www.z2labs.io](https://www.z2labs.io/)

## Summary

| case (rate / bins) | 83090c0 | PIE unpack, 1 core | **2 cores + core-0 assist** |
|---|---|---|---|
| 16 MS/s / 256 | 25 % (stride 4) | 33 % (s3) | **50 % (s2)** |
| 16 MS/s / 1024 | 16.6 % (s6) | 25 % (s4) | **50 % (s2)** |
| 16 MS/s / 2048 | 33 % (s3, some skips) | 33 % | 33 % (s3) |
| 40 MS/s / 256 | 8.4 % (s12) | 10.4 % (s10) | **20 % (s5)** |
| 40 MS/s / 1024 | 8.3 % (s14) | 8.3 % | **20 % (s5)** |
| 40 MS/s / 2048 | ~9 % (s6, skips) | ~9 % | **14.2 % (s7)** |
| 80 MS/s / 256 | 2.2 % (s48) | 2.2 % | **10 % (s10)** |
| 80 MS/s / 1024 | not allowed | not allowed | **7.1 % (s14)** |
| 80 MS/s / 2048 | not allowed | not allowed | **7.1 % (s14)** |

"Coverage" = fraction of the received samples that went through an on-chip
FFT, mean detector, runs without abandoned blocks, drops, CRC errors or
desyncs (`stride_sweep.py` criteria). The ring itself stays gapless in every
case; coverage only says how many of the gapless samples the CPU managed to
transform.

- Per-block CPU cost: 12 216 -> 7 047 cycles at 256 points (1.73x).
- Second core: 2-5x more coverage, depending on rate and FFT size.
- 80 MS/s now supports 1024 and 2048 bins (previously 256 only).
- Spectra are unchanged within measurement noise (VSG A/B below).
- 0 Hz: the per-block DC notch is replaced by a slow DC tracker, so signals
  at/near the LO stay visible (+35-46 dB instead of being swallowed).
- Table = final state with opt1-11 (`stride_up11.json`).
- The stream can carry live chip statistics (core load, coverage, heap),
  shown in the web viewer footer.

## 1. Profiling first

`s3_prof` (kept outside the firmware tree, applied with a patch script and
removed again before committing) adds two commands:

- `PROF <nfft> <iters>`: micro-benchmark of every stage on synthetic data,
  25+ items, including bit-exact checks of the optimised kernels.
- `PROF?`: cycle split of the last real SPEC run per phase (work, idle,
  pump, prep, switch, locate, unpack, fft, dc, accum, emit).

Baseline per stage (CPU cycles at 240 MHz):

| stage | 256 | 1024 | 2048 |
|---|---|---|---|
| unpack IQ10 + Hann | 4 364 | 17 420 | 34 828 |
| FFT (s3_fft2r_sc16_rnd) | 3 649 | 16 885 | 36 223 |
| accumulate (float, scattered) | 4 107 | 16 395 | 32 779 |
| **block total** | **12 216 (50.9 us)** | 50 796 | 103 926 |
| emit (per frame) | 14 370 | 57 378 | 114 722 |
| CRC32 (per frame) | 2 841 | 10 411 | 20 508 |

Findings that drove the work:

- The FFT is only 30-35 % of a block. Unpack and accumulate together cost
  twice as much as the FFT.
- At 40/80 MS/s, 22-37 % of core 0 was idle polling: a whole block did not
  fit before the next bank switch, so the CPU waited.
- Pumping the USB Serial/JTAG FIFO costs 7-12 %.

## 2. Single-core optimisations

### opt1: integer dB coding
Frame emit converted every bin with `log10f`. It now takes log2 from the
float bits: exponent plus a 7-bit mantissa lookup table, with the
mean-detector offset folded in. Max error 0.05 dB, smaller than the previous
float path's error. Emit share of a run: 8.6 % -> 5.3 %.

### opt2: accumulate in slot order
The FFT output is bit-reversed. Accumulating in natural bin order meant
scattered loads and stores. The accumulator is now indexed in FFT slot order,
and the permutation is applied once per frame in emit. The max-hold detector
uses uint32 power instead of float. Accumulate: 4 107 -> 2 315 cycles per
256-point block.

### opt3: PIE unpack + Hann (`s3_unpack.S`)
The ring holds packed IQ10 words (I in bits 0-9, Q in bits 10-19). The new
assembly kernel uses the S3 PIE (SIMD) unit:
- `ee.ld.128.usar.ip` + `ee.src.q` for unaligned 128-bit loads;
- `ee.vsl.32` and `andq`/`orq` to split and sign-extend the I and Q fields;
- `ee.vmul.s16` with SAR = 15 to apply the Q15 Hann window (pre-interleaved
  table `win2`).

Result: 4 364 -> 993 cycles per 256 samples (4.4x). It is bit-exact against
the scalar C path, checked on the chip for all 4 source alignments. It is
linked into IRAM (`linker.lf`: `s3_unpack (noflash)`).

With opt1-3 a 256-point block costs 7 047 cycles instead of 12 216 (1.73x).

## 3. The second core (opt4)

ESP-IDF stays configured as **UNICORE**. FreeRTOS, drivers and interrupts
all remain on core 0. `ring_capture_init` starts the APP CPU by hand as a bare
worker:

1. `cpu_utility_ll_unstall_cpu(1)`,
   `cpu_utility_ll_enable_clock_and_reset_app_cpu()`, and
   `ets_set_appcpu_boot_addr(s3_core1_entry)`.
2. `s3_core1.S` (follows the eSpDR startup.S pattern):
   - PS.INTLEVEL = 15, so no interrupts ever run on core 1;
   - windowed ABI, CPENABLE = FPU + PIE;
   - VECBASE = IDF `_vector_table`;
   - its own 8 KB stack;
   - then a jump to `s3_core1_main()`.
3. Core 1 runs only IRAM code and touches only DRAM data. It does no flash
   access and no IDF calls, so cache disable during flash writes cannot
   hurt it.

Work split:

- **Core 0:**
  - drives the dump engine and the 3-bank ring;
  - locates each unit;
  - posts units to core 1;
  - pumps the USB FIFO.
- **Core 1:**
  - unpack -> FFT -> DC removal -> accumulate -> frame build/CRC;
  - pushes finished frames into a single-producer/single-consumer TX queue
    that core 0 drains.

### Bank hand-off protocol
A bank must not be refilled while core 1 still reads it.

- Every posted unit carries a sequence number. `bank_seq[bank]` says which
  unit may read the bank.
- When core 0 needs the bank back, it **revokes** it: `bank_seq = 0`, then
  `memw`, then it waits while `busy == bank+1`.
- Core 1 sets `busy`, issues `memw`, and re-checks `bank_seq` before every
  read. This is a Dekker-style handshake, so neither side needs a lock or an
  atomic for it.
- Blocks core 1 could not read are counted as `abandoned` and flagged in the
  frame header (flag bit 1).

## 4. Core-0 assist (opt5)

With core 1 doing all the DSP, core 0 sits mostly idle between bank
switches. It now helps:

- Both cores claim blocks of the open unit from a shared cursor with an
  atomic compare-and-set (`xt_utils_compare_and_set`, S32C1I).
- Core 0 only claims a block if unpack + FFT is predicted to finish before
  its next ring deadline. The prediction is `c0_cost`, the longest core-0
  block seen so far, starting from a margin over the warm-up FFT time.
- Core 0 does unpack + FFT + DC removal into a hand-off buffer `hbuf`
  (2 x 2048 x int16 = 8 KB). Core 1 accumulates it, so there is still only
  one accumulator and all frame logic stays on core 1.
- When a unit closes, core 1 stops further claims and waits for a core-0
  block that is still in flight before it emits.

At 16 MS/s / 256, core 0 contributes about 30 k blocks per 2 s run.

## 5. More bins at 80 MS/s (opt6, opt7)

Single-core mode allowed only 256 bins at 80 MS/s: a longer block did not fit
between bank switches. On the worker core that limit no longer applies, so
`SPEC` accepts 1024/2048 bins at 80 MS/s whenever dual mode is active.

The first attempt failed with LATE/AGE errors:

- Core 0 had to wait while core 1 read a bank.
- A 2048-point PIE read takes about 8 k cycles.
- A block that wraps the ring end fell back to scalar code, up to about
  35 k cycles.

opt7 fixes this:

- `unpack_seg()` keeps the PIE path across the ring wrap. Only the 8 or
  fewer samples straddling the wrap go through scalar code. It was tested on
  the host for 6 896 offset/length cases, all bit-exact.
- Core 1 reads a bank in 256-sample slices and re-checks the revocation flag
  between slices. Core 0 never waits more than about 1 k cycles. A revoked
  block is dropped and counted as abandoned.

Base frame merging (`upf`) at 80 MS/s was raised so the USB link keeps up:
16 units per frame at 1024 bins and 30 at 2048 bins.

## 6. Live chip statistics (opt8)

`SPEC ... 1` (a 7th field) makes the frame producer insert an "SPS1" frame
about every 250 ms. It is 36 bytes plus CRC32 (40 bytes), little-endian:

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;               // "SPS1" 0x31535053
    uint16_t c0_pm, c1_pm;        // core load over the period, per mille
    uint16_t cov_pm, mode;        // FFT coverage per mille; bit0 dual, bit1 assist
    uint32_t heap_free, heap_largest; // internal heap before the run, bytes
    uint32_t abandoned, drops;    // run totals
    uint16_t late_max, txq_pm;    // max switch lateness (pairs), TX queue fill
    uint32_t ffts_per_s;
} spec_stats_t;
```

How the load is measured:

- Core 1 counts its busy cycles per unit.
- Core 0 counts bank preparation, USB pumping, assist blocks and single-core
  work slices.
- Both are divided by the elapsed cycles of the period.

Without the 7th field the stream is byte-identical to before, so older host
tools keep working. CAPS advertises `SPECSTAT`.

Typical readings:

| case | core 0 | core 1 | coverage | FFT/s |
|---|---|---|---|---|
| 16 MS/s / 256, s2 | 42 % | 83 % | 50 % | 31.3 k |
| 80 MS/s / 2048, s6 (before opt9/11) | 26 % | 100 % | 8 % | 3.0 k (+ ~350 k skipped) |

Free internal heap during SPEC: about 37 KB, largest block 22 KB.

## 7. Continuous stride, sliced frame encode (opt9, opt11)

Running the viewer at 80 MS/s / 2048 bins showed core 1 at 100 %, ~8 %
coverage and a fast-growing "skipped" counter. Two causes:

- **The stride restarted at block 0 in every unit** (opt9). At 80 MS/s a unit
  (12 288 pairs, ~154 us) holds only 6 blocks of 2048, so any stride >= 6
  still asked for one FFT per unit (16.7 %). Core 1 manages ~3 k FFT/s
  (~60 k cycles per 2048 block), so half of the requested blocks were
  abandoned at every stride: ~8.7 k per 2.5 s, regardless of the setting.
  Core 0 now carries the stride phase from unit to unit (dual and
  single-core paths), so stride 14 really means every 14th block. A frame is
  only closed once it holds at least one FFT (or after 4x upf units).
- **Frame encode blocked core 1 for ~3 units** (opt11). Encoding + CRC of a
  2048-bin frame ran in one go; units queued meanwhile lost their bank. The
  result was exactly one abandoned block per frame (~540 per 2.5 s at upf
  30), independent of the stride. The accumulator is now double-buffered
  (2 x 8 KB). Closing a frame writes the header and swaps buffers. The bins
  are encoded 256 at a time and the CRC 1 KB at a time, one slice after each
  unit and in idle polling. A new frame close finishes a pending one first.

After both fixes 80 MS/s / 1024 and 2048 run clean at stride 14, and 40 MS/s
gained a step at 1024 (s6 -> s5) and became clean at 2048 (s7).

## 8. 0 Hz: DC tracker instead of a notch (opt10)

The ESP32-S3 front end is zero-IF, so 0 Hz carries the LO leakage / DC
offset and its slow wander. The old `remove_dc` zeroed bin 0 of every block
and cancelled its Hann leakage into bins +-1. That removes the offset
perfectly, but it also removes **every signal within ~1 bin of the LO**, and
at 80 MS/s / 256 a bin is 312 kHz wide.

Raw captures (RINGCAP, 16 and 80 MS/s, gain 50, no signal) show:

- DC offset ~5 LSB on Q, stable between captures.
- Block-to-block wander ~0.7 LSB.
- Noise RMS 3.5 LSB.

New default (`DC 1`):

- A running complex average of X[0] (1/64 per block, shared by both cores,
  reset per run) estimates the static offset.
- Only that estimate is subtracted from bin 0, and its Hann leakage
  (estimate / 2) from bins +-1.
- `DC 0` restores the old notch.

On-chip SPEC A/B with a VSG60 CW at the LO + offset, −50 dBm, levels in dB
above the median floor:

| case | no signal, bin 0 | CW at +0 / +5 kHz | CW at +20 kHz |
|---|---|---|---|
| 16 MS/s / 1024, notch | hole (−12) | 18-19 | 33 |
| 16 MS/s / 1024, tracker | 13.6 | **35** | 35 |
| 80 MS/s / 256, notch | hole (−10) | 13-14 | 15 |
| 80 MS/s / 256, tracker | 20 | **38** | 38 |
| 80 MS/s / 2048, notch | hole (−2) | 20-22 | 38 |
| 80 MS/s / 2048, tracker | 25 | **46** | 45 |

The trade-off: with the tracker the empty band shows a residual spike at
0 Hz (the DC wander, +14 to +25 dB), but a signal there is ~20 dB above
that spike instead of disappearing. The spike itself is the zero-IF
architecture and cannot be removed without also removing signals at the LO.
The complete answer is offset tuning (LO placed beside the frequency of
interest); the viewer has it as "0 Hz: offset LO".

The spike is not IQ imbalance:

- The PHY's own RX IQ calibration is good. A CW at LO+3 MHz (16 MS/s,
  −50 dBm) put its image below the noise floor: IRR > 39 dB, limited by the
  floor.
- The DC offset moves with the input. It was (0, +5) LSB with no signal and
  (−7, −13) LSB with the −50 dBm CW present, which is LO self-mixing. A
  one-time calibration with a generator would therefore not hold, and the
  running tracker is the right tool.

## 9. Commands

| command | reply / effect |
|---|---|
| `SPEC <ms> <stride> <upf> <mode> <rate> <nfft> [stats]` | as before; `stats=1` inserts SPS1 frames |
| `DUAL?` | `DUAL <mode> <core1_alive>` |
| `DUAL 0` | single core (previous behaviour) |
| `DUAL 1` | core 1 worker + core-0 assist (**default**) |
| `DUAL 2` | core 1 worker, no assist |
| `ASSIST?` | `ASSIST <on> <blocks core 0 handed off in the last run>` |
| `DC?` / `DC 0` / `DC 1` | 0 Hz handling: notch / slow DC tracker (**default**) |

CAPS gains `SPECSTAT` and `DCT`. If core 1 fails to start, `DUAL?` reports
`core1_alive = 0`. SPEC then falls back to the single-core path, and the
80 MS/s bin limit applies again.

## 10. Web viewer (esp-web-sdr)

The viewer takes the stride and base units-per-frame from `SPECINFO?`.
With the second core active, the S3 reports these profiles:

| rate | 256 bins | 1024 bins | 2048 bins |
|---|---|---|---|
| 16 MS/s | s2, upf 1 | s2, upf 4 | s3, upf 7 |
| 40 MS/s | s5, upf 3 | s5, upf 9 | s7, upf 17 |
| 80 MS/s | s10, upf 5 | s14, upf 16 | s14, upf 30 |

With `DUAL 0` it reports the previous single-core profiles. On top of that:

- On the S3, frames are merged to the waterfall row time:
  `upf = max(profile upf, floor(rowMs / unitMs / 2))`.
  - Example, 16 MS/s / 256 at 10 ms rows: 1300 -> ~217 frames/s.
  - Emit + pump CPU drops from 12.4 % to 3.1 %.
- The on-chip spectrum is selected automatically after connecting when the
  firmware offers it. Choosing I/Q streaming sticks for the page.
- Auto-connect:
  - the chosen port's USB VID/PID is remembered (localStorage `espSdrPort`);
  - the viewer connects on page load and on USB plug-in;
  - the first-time picker is filtered to Espressif native USB (0x303A);
    Shift+click shows all ports.
- SDR++-style display zoom:
  - mouse wheel zooms around the cursor;
  - left-drag pans;
  - double-click returns to the full span;
  - a plain click still tunes;
  - the waterfall keeps its rows and is redrawn for the new view.
- Interactive frequency axis:
  - wheel zooms, drag pans, double-click resets;
  - 1-2-5 major/minor ticks, and the spectrum grid follows them;
  - markers for the LO (0 Hz) and the tuned frequency.
- Dragging past the edge of the received span retunes the LO on release:
  whole MHz steps, within `RANGE`, with the view and zoom kept.
- "0 Hz (zero-IF)" mode:
  - show as is (default);
  - fill DC bins;
  - offset LO: the LO is placed 2 MHz (16 MS/s) or 5 MHz (40/80 MS/s)
    below the tuned frequency, keeping it off the DC spike.
- A draggable splitter sets the spectrum/waterfall ratio. The off-band
  warning is an amber frequency field with a tooltip.
- Auto scale:
  - floor in 5 dB steps with 4 dB hysteresis;
  - top from a slowly decaying peak hold;
  - updated once per second.
- The footer shows "FFT on X % of samples" and the SPS1 statistics:
  core 0 / core 1 load, coverage and FFT/s, free RAM / largest block,
  late, skipped blocks, TX queue fill.

## 11. Verification with a CW reference (VSG60)

Setup:

- CW at 2444 MHz, −50 dBm, antenna 2 cm from the board;
- LO 2443 MHz, gain 45;
- 16 MS/s / 1024, mean detector;
- multiple runs per firmware.

| comparison | peak diff | per-bin mean diff | stream errors |
|---|---|---|---|
| 83090c0 vs opt1-3 | ≤ 0.13 dB | −0.2 … +0.05 dB | none |
| 1 core vs 2 cores (same session) | ≤ 0.27 dB | −0.17 … +0.07 dB | 0 CRC, 0 desync |

There is about 0.6 dB of level drift between morning and afternoon sessions
(temperature / setup). It is identical for both firmwares.

## 12. Known limits / next steps

- Core-1 block time varies: max about 13 k cycles against a mean of about
  7.7 k at 256 points. The likely cause is SRAM/IRAM bank contention with
  the dump engine and core 0.
- A single hand-off buffer limits the assist: core 0 skips while `hbuf` is
  full. Two buffers should push 16 MS/s toward 60-70 %.
- At 80 MS/s / 1024-2048 core 1 is compute-bound (~3 k FFT/s at 2048, about
  100 % load). More coverage there needs a faster FFT or core 0 doing
  FFT stages across its idle gaps: a whole 2048 block does not fit between
  two bank switches.
- `late_max` sits at ~900 pairs at 80 MS/s. That is below the 2000 limit,
  but the cause is not yet known.
- Free internal heap is about 37 KB before opt11; the second accumulator
  takes 8 KB of it. A 4096-point FFT would not fit without trimming.
- Offset tuning in the viewer would keep the frequency of interest away
  from the 0 Hz spike.
- Ideas not yet tried:
  - a 4096-point FFT;
  - a zoom FFT (DDC + decimation) for fine RBW;
  - less frequent USB pumping or USB-OTG DMA;
  - the EE.FFT.* instructions.

## Tuning range

A VSG60 CW sweep from 0.15 to 6 GHz (LO 4 MHz below the CW, 16 MS/s) found:

- The PLL locks for LO 2196-2806 MHz. Both edges are hard (no response to
  −8 dB within 10 MHz), so this is the VCO range, not a filter.
- Inside that range the chain response is within 3 dB from 2240 to
  2470 MHz, −10 dB near 2570 MHz and −21 dB at 2800 MHz (relative to
  2440 MHz).
- There is no reception outside the range, including 5.8 GHz.

`RANGE?` on the S3 now answers `RANGE 2200 2800 1` instead of 100-6000.
Wider coverage would need an external converter.

## Files

- `main/common/ring_capture.c`, behind `CONFIG_IDF_TARGET_ESP32S3` where
  S3-specific:
  - opt1-11: integer dB coding, slot-order accumulation, PIE unpack;
  - core-1 worker, hand-off and assist;
  - sliced bank reads, SPS1 stats, stride phase, DC tracker, sliced encode.
  - The DC tracker and the continuous stride are shared by all targets.
- `main/common/ring_capture.h`: `stats` config field, `ring_capture_dc_mode`,
  and the S3 dual/assist API.
- `main/targets/esp32s3/s3_unpack.S`: PIE unpack + window.
- `main/targets/esp32s3/s3_core1.S`: core 1 entry.
- `main/targets/esp32s3/receiver.c`:
  - `DUAL`, `ASSIST`, `DC`;
  - the SPEC stats field and CAPS `SPECSTAT DCT`;
  - dual-core `SPECINFO` profiles;
  - the 80 MS/s bin rule;
  - `RANGE 2200 2800`.
- `main/targets/esp32s3/linker.lf`: `s3_unpack (noflash)`.
- `main/CMakeLists.txt`: the two new assembly sources.

Memory: `sram_guard.ld` requires `.bss` to end below the RF ring
(0x3fcb0000). The esp-dsp twiddle table and the core-0 hand-off buffer
(8 KB) therefore come from the heap, which leaves `.bss` ending at
0x3fcada58.
