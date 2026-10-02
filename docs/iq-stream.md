# Continuous IQ stream (ESP32-S3)

The S3 can stream gapless, decimated complex baseband over native USB. The
16 MS/s ring is filtered and decimated on core 1 (the same worker as SPEC), so
the host receives a few hundred kS/s of real IQ instead of spectra. Hosts such
as [esp-sdr-bridge](https://github.com/z2labs/esp-sdr-bridge) serve it to
SDR++, SDR#, gqrx or GNU Radio (SpyServer / rtl_tcp).

## Commands

Send `IQS <milliseconds> <decimation> <bits> <rate_code> [shift] [mode]`.

| Field | Meaning |
| --- | --- |
| milliseconds | Run time; 0 runs until any byte from the host (or until output has been blocked for 2 s) |
| decimation | 64 to 1024, power of two (16 MS/s: 250, 125, 62.5 ... 15.6 kS/s) |
| bits | 4, 8 or 16 bits per I and Q component |
| rate_code | Ring rate code (0, 1 or 6) |
| shift | Rounding right shift of the filter output; default `15 - bits` (0 for 16 bits) |
| mode | 0: filter at 0 Hz IF; 2: multiply by j^n (+fs/4 shift) before filtering |

Native USB only (`ERR transport` otherwise). The start reply is
`IQS <sample_rate_hz> <decimation> <bits> <shift> <mode> <MHz>`, then binary
frames follow, then an `IQSEND` line with the same twelve fields as
`SPECEND`; its `ffts` field carries the number of input pairs lost to
abandoned work.

`FOFS <kHz>` sets a signed PLL offset that is applied from the next tune (also
on Wi-Fi channel frequencies), giving 1 kHz tuning steps. `FOFS 0` restores
the default.

Mode 2 is meant for a LO tuned fs/4 (4 MHz at 16 MS/s) below the wanted
centre: the LO leakage and the 1/f hump at 0 Hz IF then fall outside the
output band. As with the raw ring, RF above the LO appears at negative
frequency; hosts conjugate the samples.

## Frames

All integers are little-endian:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | ASCII `IQS1` |
| 4 | 4 | Frame sequence, from 0 for each run |
| 8 | 8 | Output sample index of the first sample |
| 16 | 2 | Samples in the frame |
| 18 | 1 | Bits per component |
| 19 | 1 | Flags: bit 0 gap before this frame; bit 1 frame(s) dropped before this one |
| 20 | 2 | Decimation |
| 22 | 1 | Gain metadata from the first I/Q word |
| 23 | 1 | Shift |
| 24 | N | Interleaved I, Q (int8 or int16; 4 bits: I in the low nibble, Q in the high nibble), N <= 1024 |
| 24+N | 4 | CRC32 (zlib) of header and payload |

The sample index advances by the frame's sample count; any jump is a gap the
host can measure exactly. Gaps occur only when the host stops reading (the
output queue overflows, flag bit 1) or when the worker has to abandon a bank.

## Signal path

1. Ring words are unpacked into separate I and Q int16 arrays, scaled by 64
   (`ee.vunzip.16`), and optionally rotated by j^n.
2. Stage 1 decimates by 8 with 32 taps: an 8-sample boxcar squared (triangle)
   convolved with an 18-tap Kaiser low pass. It has double zeros at every
   multiple of fs/8, exactly the bands that alias onto 0 Hz.
3. Stage 2 decimates by decimation/8 with up to 256 taps (Kaiser, beta 5.65,
   pass band 0.4 and stop band 0.6 of the output rate).
4. The output (10-bit sample x 32) is shifted with round-to-nearest and
   saturated to the requested bits.

Both stages use `ee.vmulas.s16.accx` on split arrays; the cost is about 12
cycles per input pair on core 1. Without the second core the work runs in
slices on core 0, which is only sufficient at lower ring rates.
