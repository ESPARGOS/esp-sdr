# On-chip spectra

| Chip | Continuous RF capture with on-chip FFT | Snapshot FFT |
| --- | --- | --- |
| ESP32 | — | 256–2048 bins; 16/40/80 MS/s; UART |
| ESP32-C2 | — | 256–2048 bins; 80/40/16 MS/s; UART |
| ESP32-C3 | — | 256–2048 bins; 16/40/80 MS/s; USB or UART |
| ESP32-C5 | 256 bins; 4/8 MS/s; native USB | All other profiles; 256–2048 bins over UART |
| ESP32-C6 | 256 bins; 16/40/80 MS/s; native USB | 512–2048 bins over USB; 256–2048 over UART |
| ESP32-H2 | — | 256–2048 bins; 6.4/10.667/16/32 MS/s; USB or UART |
| ESP32-C61 | 256 bins; 4/8/10/20/40/80 MS/s; native USB | 512–2048 bins over USB; 256–2048 over UART |
| ESP32-S2 | — | 256–2048 bins; 16/40/80 MS/s; USB or UART |
| ESP32-S3 | 256–2048 bins; 16/40/80 MS/s; native USB | — |
| ESP32-S31 | 256–2048 bins; 4/8/10/20/40/80 MS/s; native USB | Same FFT sizes and rates over UART |

Continuous capture keeps the RF writer running, but the CPU analyzes only
selected FFT windows: it does not deliver every sample. Snapshot FFT stops
reception between captures, and the viewer marks these gaps.

`CAPS` advertises `SPEC SPECN SPECCAPS` only on enabled targets. Query
`SPECINFO?` before enabling a control. Its reply is `SPECINFO <JSON>`:

```json
{"continuous":false,"transports":["USB","UART"],"profiles":[[80000000,0,256,1,1,1],[80000000,0,512,1,1,0]]}
```

C2 supports UART snapshot spectra at 80/40/16 MS/s (codes 0/1/6),
with 256/512/1024/2048 bins at each rate.

H2 uses rate codes 7/6/8/9 for nominal 32/16/10.667/6.4 MS/s. Code 8 is
32 MHz / 3, represented as 10666667 Hz in integer protocol fields. Existing
codes 0–7 retain their meanings. H2 provides snapshot spectra at
256/512/1024/2048 bins on all four rates over USB or UART.

Each profile contains `[sample_rate_hz, rate_code, fft_bins, stride,
units_per_frame, continuous]`. The last field is optional for compatibility;
five-field profiles inherit the top-level `continuous` value. Capabilities
are specific to the connection: C5/C6/C61 use snapshot FFT on UART and
continuous 256-bin capture on native USB (C5 at 4/8 MS/s only). S31 supports continuous capture at
all advertised FFT sizes and rates on native USB, with snapshots over UART.
S3 spectra require native USB.
Unsupported controls stay hidden. Existing S3 firmware without `SPECCAPS`
uses the original S3 compatibility profiles.

`SPEC` always processes as many FFTs as its capture and transport allow.
Completed powers accumulate while the previous frame is being transmitted;
once output is available, it emits and clears that batch. There is no fixed
frame timer, rolling history, or host request. The legacy stride and batch
arguments remain in the command for compatibility, but no longer limit SPEC
processing. Mean versus max-hold remains selectable.

Send `SPEC <milliseconds> <stride> <units_per_frame> <detector> <rate_code>
<fft_bins> [stats]`. When `CAPS` includes `SPECSTAT`, append `1` to receive
statistics alongside spectra (omitting it preserves the original protocol). Zero milliseconds runs until a stop byte; detector 0 means mean
power and 1 means maximum power. Use the parameters from the chosen profile.
Portable snapshots require stride 1. Incomplete FFTs are excluded from batches.

The start reply is `SPEC <fft_bins> <sample_rate_hz> <unit_pairs> <MHz>`.
Binary frames follow. All integers are little-endian:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | ASCII `SPC1` |
| 4 | 4 | Frame sequence |
| 8 | 8 | First sample index |
| 16 | 4 | Samples spanned by the frame |
| 20 | 2 | Completed FFTs represented |
| 22 | 1 | Flags: bit 0 maximum detector; bit 1 skipped processing; bit 2 prior output drop; bit 3 snapshot gaps |
| 23 | 1 | Gain metadata from the first I/Q word |
| 24 | 2 | Saturating cumulative frame-drop count |
| 26 | 1 | log2(FFT bins) |
| 27 | 1 | dB code multiplier (2) |
| 28 | N | Power codes in natural FFT order |
| 28+N | 4 | CRC32 of header and bins |

A power code represents `20*log10(power)` in the normalized Q15 FFT domain,
clamped to 0–255. The viewer converts this to dBFS using the Hann-window
normalization. Zero is a quantized floor, not evidence of absent RF energy.
Snapshot sample indices use elapsed wall time. Bank-rotation indices
come from the measured contiguous bank boundaries.

The end line has twelve decimal fields:

```
SPECEND status detail units pairs elapsed_us late_max work_max_cycles frames drops abandoned ffts stopped_by_host
```

Normal stops return status 0. A bank deadline/continuity failure stops capture
instead of silently presenting broken timing. Empty S3 frames are dropped,
including a final stretch in which no FFT was selected.
A host that stops reading can lose a partial frame and the end record when
bounded output timeouts expire. The decoder resynchronizes on CRC-valid frames
or an end record; if it cannot establish the end boundary, it marks the
connection failed and requires reconnecting. It must not issue ordinary
commands into that uncertain stream. Reconnection reacquires the serial lease.

`SPS1` statistics are optional 40-byte records, emitted approximately every
250 ms. They have a 36-byte little-endian body followed by its CRC32:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u32 | ASCII `SPS1` |
| 4, 6 | u16 | Core 0 / core 1 processing load, per mille |
| 8 | u16 | Fraction of samples analyzed by FFT, per mille |
| 10 | u16 | Bit 0: dual core; bit 1: core 0 assists FFTs |
| 12, 16 | u32 | Free internal heap / largest block before capture, bytes |
| 20, 24 | u32 | Cumulative abandoned work / dropped frames |
| 28, 30 | u16 | Maximum bank lateness in samples / output queue fill per mille |
| 32 | u32 | FFTs per second |

Load measures time spent processing captures, not general FreeRTOS CPU usage.
Snapshot coverage includes gaps between captures. The heap figures are taken
before interrupts are masked; they are not live allocator measurements.

All targets advertise `DCT`: `DC?` queries the on-chip DC correction;
`DC 1` selects the default slow tracker (1/64 update per FFT), and `DC 0`
removes each FFT's instantaneous DC estimate. Both correct the Hann window's
adjacent bins. The tracker resets at each capture and can also suppress a
stationary signal exactly at the LO. The viewer's offset-LO setting moves the
frequency of interest away from this correction; “fill DC bins” only changes
the display. Raw I/Q captures retain their original behavior.

## How each chip computes spectra

**Snapshot chips (ESP32, S2, C2, C3, H2; C5 fallback profiles):** capture a burst, stop the
dump engine, give the SRAM back to the CPU, then run the FFTs. There is a gap
in the RF data while this happens. The C3 has to work this way at every FFT
size: while the dump engine owns its SRAM, CPU reads return a repeating
four-word pattern, which would show up as false peaks.

**C5, C6 and C61:** the dump engine keeps writing a ring of two SRAM banks. While
it fills one bank, the CPU copies a 256-sample window out of the other in one
bounded step and runs a scalar FFT on it in short slices. The FFT, checksum and
statistics code is warmed up before capture starts.

C5 runs the capture worker on an LP-SRAM stack because selecting its second
destination temporarily makes live heap/ROM data CPU-inaccessible. Snapshot
processing reuses reserved SRAM after the finite writer stops. See
[C5 capture notes and hardware validation](esp32c5-continuous.md).

**S3:** a ring of three banks. Core 1 runs a bare-metal worker that unpacks
samples with SIMD instructions, computes FFTs, accumulates power and encodes
frames. Core 0 rotates the banks, feeds USB, and helps with FFTs when it has
time. A bank is only handed back to the dump engine once nobody is reading it.
`DUAL 0` runs everything on core 0, `DUAL 1` (default) uses the worker with
core 0 helping, `DUAL 2` uses the worker alone; `DUAL?` reports the mode and
whether the worker started. Query `SPECINFO?` again after changing it.

**S31:** two 128 KiB groups of SRAM banks rotate without stopping the dump
engine. Core 0 checks bank boundaries and handles USB; core 1 does SIMD
unpacking, windowing and FFTs, and skips work it cannot finish before the
bank is reused.

The continuous modes mask interrupts while they run and therefore disable the
interrupt and task watchdogs in those firmware profiles; capture and host-stall
timeouts end a run instead. Their hot code and buffers live in internal RAM to
avoid flash-cache stalls. When processing can't keep up, work is skipped and
counted (`abandoned` in the end report, `SPS1` statistics), never silently
dropped.

The shared snapshot code accumulates power and peak values in integer
arithmetic (mean accumulation in floating point), and both detectors share
one workspace.

## Testing

`tools/check_spectrum.py` runs every advertised profile with both detectors on
an attached board, checks frame CRCs, end reports and command access
afterwards, and prints a JSON report:

```sh
python tools/check_spectrum.py --port /dev/ttyACM2 --milliseconds 3000 --stats
```

All profiles pass on ESP32, C2, C3, C5, C6, C61, H2, S2, S3 and S31 over every
interface each chip offers. Continuous capture is verified with hardware sample
indices and bank boundaries, not with RF phase measurements. The S31's SIMD
unpacking and FFT were also checked against a scalar reference (within three
output counts for zero, tone and random inputs at all sizes).
