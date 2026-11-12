# ESP32-C5 continuous capture

Native USB `SPEC` requests with 256 bins at 4 or 8 MS/s use the shared
two-bank capture engine. Other rates, larger FFTs and UART requests use
snapshots. Query `SPECINFO?` on the active transport for per-profile flags.
Continuous RF capture has partial FFT coverage, as on C6/C61; it does not
mean continuous raw I/Q output or that every sample is transformed.

## Writer and ownership

C5 uses the C61 dump registers and rate codes, with the C6 bank map:

- Source 15 in `0x600a9008[20:17]`, rate code in `[23:21]`.
- A 16,384-word circular writer, low 14 write-index bits.
- Clear/start `0x600a9004` with `0x24000 | BIT(18)`, `0x24000`,
  then `0x80024000`. Do not pulse the finite-capture trigger, bit 19:
  it stops this circular configuration after a short capture.
- Ownership register `0x60095004`: bits 9/10 select the two destinations;
  bit 16 remains clear to write each bank's lower 64 KiB.

CPU and AHB-GDMA reads from an RF-owned bank return stale bus values.
Stopping the writer alone does not restore CPU access. Rotate ownership
instead, and process the previously completed bank while RF fills the next.
The shared engine checks matching sentinel boundaries and fails explicitly
on late/invalid handoffs.

| Memory | Use |
| --- | --- |
| Below `0x40820000` | Application static data/code, guarded by existing linker assertions |
| `0x40820000..0x40840000` | Reserved bank 1; circular destination in lower half, finite captures in upper half |
| `0x40840000..0x40850000` | Additional reserved 64 KiB circular destination |
| `0x40850000..0x40860000` | Live heap/ROM data; never a writer destination |
| LP SRAM | Dedicated 8 KiB worker stack |

Ownership affects the whole 128 KiB bank, including bank 2's live upper
half. The command task's stack may reside there. `continuous.c` copies the
configuration/result through static storage and runs the engine on LP SRAM.
While ownership rotates, interrupts stay masked and the engine accesses
only its bank-safe state, LP stack and CPU-owned sample destination.
Ownership is restored before normal scheduling resumes.

The ordinary snapshot FFT/output workspace occupies bank 1's unused lower
half, outside the finite writer destination. It is accessed only after
finite capture restores CPU ownership. Each snapshot request rebuilds its
window because a preceding circular run may have overwritten it. Keeping
both maximum-size snapshot and ring workspaces in BSS would exceed the
guarded static region; allocating the snapshot workspace from the remaining
heap failed at 2,048 bins in hardware testing.

Wi-Fi packet-buffer counts are reduced to leave heap for receiver startup.
As for other bank-rotation targets, interrupt/task watchdogs are disabled:
the engine uses bank deadlines, requested duration and host-stall checks.
An otherwise working 60-second run reset with `TG0_WDT_HPSYS` before the
task-watchdog default was corrected.

## Validation (2026-11-12)

Hardware: one Waveshare ESP32-C5-LCD-1.47, ESP32-C5HF4 revision v1.0,
48 MHz crystal, 4 MB flash, no PSRAM; CPU at 240 MHz, receiver at 2412 MHz.
Pinned ESP-IDF: `25fe69f946311abdaf9ad56591f25fedbc20ac98`.

- Production C5 cross-build passes the existing SRAM assertions.
- All 24 advertised USB profiles, both detectors: 48 successful 300 ms
  captures, including snapshot sizes through 2,048 bins and transitions
  between snapshot and circular modes. Frame/telemetry CRCs, sample-index
  ordering, end status and subsequent command access passed.
- Four consecutive 60-second circular captures passed:

| Rate | Detector | Units / frames / FFTs | Captured pairs | Elapsed (us) | Maximum handoff lateness (pairs) |
| --- | --- | --- | --- | --- | --- |
| 8 MS/s | Mean | 39,049 | 480,009,932 | 60,001,248 | 2 |
| 8 MS/s | Max | 39,049 | 480,009,871 | 60,001,240 | 2 |
| 4 MS/s | Mean | 19,528 | 240,008,825 | 60,002,214 | 1 |
| 4 MS/s | Max | 19,528 | 240,009,627 | 60,002,414 | 1 |

All four end reports had status/detail zero, zero dropped frames and zero
abandoned units. Telemetry reported approximately 2% FFT coverage. Captured
pair counts describe the writer stream, not raw I/Q transmitted to the host.
The original board firmware was restored and its command response verified
after testing.

Reproduce with the normal C5 build and native USB:

```sh
python tools/check_spectrum.py --port PORT --milliseconds 300 --stats
python tools/check_spectrum.py --port PORT --milliseconds 60000 --stats --continuous-only
```

Host snapshot regression coverage includes C5's reserved workspace and
USB/UART profile flags, alongside C3/C61. On macOS the full host suite has
ten existing calibration subtest errors because Apple's linker does not
support GNU `--wrap`; these also reproduce on the untouched upstream base.

These are bounded measurements on one board and frequency. Sentinel/index
continuity is not an external RF phase measurement. Other chip revisions,
5 GHz circular capture, UART hardware fallback, LCD coexistence, higher
circular rates and unlimited-duration stability are not established by
these results.
