# Receive controls

Protocol 6 advertises `RXLIMITS` in `CAPS`. Query `LIMITS?` after `INFO` for:

- `gain`: `[minimum, maximum, step]` in PHY table indices.
- `bandwidth`: `[minimum, maximum, step, default]` in MHz, or `null` when no
  characterized MHz mapping is available.
- `rates`: supported nominal sample rates in samples/second.
- `bits`: supported precision per I/Q component.

Hardware AGC is the default; manual gain indices are not absolute gain in dB.

## Gain implementation

Gain maxima come from the calibrated PHY tables where available. C61 and S3
read AGCPWR_CTRL7 bits 8–14; C5 uses its PHY gain-table setup; S31 snapshots
its generated table. H2 reads the calibrated maximum from `phy_param[83]`
and keeps Bluetooth RX forced on when returning to hardware AGC.

C61 manual gain follows `sensor-firmware/main/iq/modem.c`: mirror the forced
low-table entry into slot index+80, set AGC initial gain/threshold, and disable
RF saturation intervention. Restore overwritten entries and calibrated settings
when returning to hardware AGC. The mirror prevents DSSS classification from
selecting an uncalibrated second-table entry.

## Bandwidth implementation

`BANDWIDTH <MHz>` interpolates the chip's capacitor-code curve in
`main/common/rx_bandwidth.h`. Zero selects the widest setting, not filter bypass.
Out-of-range requests are rejected.

| Chip | Approximate bandwidth | BBTOP registers | Code width |
| --- | --- | --- | --- |
| H2 | 4–11 MHz | 0 | 7 bits |
| C2 | 12–20 MHz | 4/5 | 6 bits |
| ESP32 | 12–67 MHz | 1/2 | 7 bits |
| C5 | 11–48 MHz | 6/7, two PHY modes | 6 bits |
| C6 | 12–54 MHz | 4/5 | 6 bits |
| C61 / S31 | 13–54 MHz | 4/5 | 6 bits |
| S2 | 15–60 MHz | 4/5 | 6 bits |
| S3 | 13–69 MHz | 4/5 | 6 bits |

The register mappings use BBTOP block 0x67, I2C host 1. Capture code preserves
unrelated bits and restores calibrated registers before transfer or retuning,
including on capture errors. C61's curve derives from the reference sensor
firmware; other chips retain their own curves.

These are approximate receive-path noise widths. Board calibration, operating
conditions and digital filtering affect them; they do not guarantee alias-free
reception at every sample rate. C5 uses PHY mode 0 for 11–23 MHz and mode 1
for 24–48 MHz or open. Changing modes runs the complete channel calibration,
including PBUS analog-control tables; retuning preserves the chosen mode. Its
curves include the digital-filter response.
ESP32's widest settings exceed the characterized span;
its numeric maximum uses code 8, while wide open selects code 0.

## Receive calibration

ESP-SDR uses the following calibration policy on all supported ESP32 families:

- **IQ imbalance:** Calibrate at startup and reuse those corrections across
  frequency changes. ESP-SDR does not repeat the loopback IQ sweep on retunes.
  H2 retains four startup corrections, one for each RF gain group.
- **DC offsets:** Remeasure at each new tuning frequency. If the PHY replaces
  DC calibration in the background, remeasure at the requested frequency
  before the next capture, even without another tuning command.
- **Repeated frequency requests:** Requesting the same frequency does not
  force another DC measurement unless the calibration has become stale.

This describes calibration controlled by ESP-SDR. The PHY can still perform
its own startup and background calibration, so “IQ calibration only once”
is not an absolute guarantee for everything inside the chip. Startup and
background calibration can still transmit; removing the retune IQ sweep does
not make the whole firmware RF-silent.

DC-measurement counters detect changes even when the PHY's temperature stamp
stays unchanged. ESP32 and S2 compare their cached DC tables instead because
internal PHY calls bypass the measurement wrappers. Recovery restores the
receive settings and uses the PHY mutex to serialize DC measurement with
tracking. The mutex is not held throughout acquisition: tracking can affect a
capture already in progress, with recovery at the next capture preparation.
S31 streaming stops acquisition, discards old DMA data, and starts a new stream
epoch on recovery. H2 and both S31 profiles currently disable periodic PLL
tracking; explicit PHY DC measurements still invalidate calibration.

## Rates and extended tuning

C6 currently exposes only nominal 80 MS/s. Other tested clock/divider settings
and dump sources did not establish a reliable lower-rate I/Q path. Unsupported
rates are rejected.

All supported burst targets advertise `TUNEEXT` and answer `RANGE?` with
`RANGE 100 6000 1`: every integer MHz from 100 through 6000 is accepted for an
attempt. Fractional MHz and values outside that software range are rejected.
`main/common/rx_tuning.h` defines the shared limits. PLL lock is not a condition
for accepting a tuning command.

C5 programs the requested frequency through `phy_set_chanfreq` after the DC
measurement. Other backends may select a standard channel before direct PLL
programming; S31 also keeps arbitrary frequencies out of channel calibration.

### Experimental lower-band LO conversion

ESP32, S2, S3 and C3 select CKGEN `0x65:0[4]`; C2 and C6 select
`0x62:16[3]` (host **1**). Both selectors provide **5/6** conversion at
1842–2209 MHz. `FREQ` still specifies the receive frequency: for example,
`FREQ 2001` programs a 2401.2 MHz PLL coordinate. Calibration runs with the
normal divider; the alternate divider is applied after RX setup. Other
frequencies restore normal conversion. Only the selector bit is changed.

The original ESP32 needs the SDK's patched `ram_chip_i2c_*` functions and
CKGEN host **4**; S2/S3/C3 use host **1**. ESP32 also needs its channel's fixed
capacitor released before a direct retune, and its offset argument is in
1/1024 MHz rather than kHz. S2 needs a bounded 512-code capacitor search:
it holds the midpoint of the widest RFPLL voltage-window interval, restoring
its previous capacitor fields if none is found. A successful command still
means a tuning attempt, not a guaranteed PLL lock.

This follows the [eSpDR S3 investigation](https://github.com/h0m3us3r/eSpDR/commit/f279bf823eee41796dfd1ac21f13e1ed9b418c82).
The older eagletest `set_freq_test()` calculation implies 8/9; external-tone
measurements on the tested ESP32, S2, S3, C2, C3 and C6 instead confirm **5/6**.
Tests use a HackRF source through antennas, source-on/off comparisons,
known tone offsets, fractional PLL settings, and returns to normal tuning.
They establish reception at discrete frequencies, not calibrated sensitivity
or guaranteed performance throughout the interval. Sample-clock configuration
is unchanged; burst timing and tone positions were checked at nominal 80 MS/s.

C5, C61, H2 and S31 were also checked. Their analog register layouts differ:
C5 places CKGEN in block `0x68`, while C61/S31's SDK resets CKGEN through
block `0x62`. The legacy `0x65:0[4]` selector is not exposed as on the older
chips, and no equivalent mode has been qualified. They keep their existing
frequency programming; do not apply the old bit or frequency multiplier to
them. The shared `rx_lo.h` deliberately accepts only the six verified chips.

The browser negotiates ranges for every chip and uses them for text entry and
spectrum click-to-tune. Older firmware retains its advertised limits, with
legacy fallbacks only when it does not advertise `TUNEEXT`. An informational
warning appears outside 2400–2483.5 MHz; C5 also excludes its 5150–5895 MHz Wi-Fi band from the
warning. This never blocks tuning.

The 100–6000 MHz expansion is host-test/build verified only. The historical
hardware checks below covered the previous 2100–2800 MHz range; they do not
validate the new endpoints or RF performance.

ESP32 validation on an ESP32-D0WD-V3 rev. 3.1 with a 40 MHz crystal covered
all 701 whole-MHz settings with CRC-checked captures, plus 132 maximum-size
captures across 80/40/16 MS/s, 8/10-bit packing and manual gain/AGC. This
checks command and capture stability, not RF accuracy or PLL lock.

C61 validation on an ESP32-C61HR2 rev. 1.0 with a 40 MHz crystal likewise
covered all 701 settings with CRC-checked captures, plus 264 maximum-size
captures across all six rates, 8/10-bit packing and manual gain/AGC. RF
accuracy and PLL lock across the extended range remain unverified.

All advertised rates are nominal. Capture timing and payload checks do not
replace independent RF/sample-clock calibration.

## Capture memory

ESP32 reserves a 64 KiB SRAM aperture at `0x3ffe8000`, with linker guards and
DPORT MAC_DUMP_MODE=3. Mode 2 only fills half the buffer. Packing occurs in place.

S2 reserves 48 KiB at `0x3fff0000–0x3fffc000` and its IRAM aliases, leaving the
top bank accessible to ROM USB. Its 12,284-sample maximum leaves four overrun
canaries. Source 0 supplies signed 10-bit I/Q; clock bits 15/16 select nominal
40/16 MS/s from the 80 MS/s source.

## GPIO outputs

Burst firmware advertises `GPIO` in `CAPS`. `GPIO?` returns a single line of
available output pins and their configured states, ordered by chip GPIO number:

```text
GPIO 0:Z 1:Z 2:Z 3:0 4:Z 14:1
```

The example is illustrative; clients must use the returned list rather than
infer pins from the chip name. An empty list is returned as `GPIO`.

`GPIO <pin> <Z|0|1>` changes one pin and replies `OK GPIO <pin> <state>`.
Invalid/unavailable pins, invalid states, and malformed requests return
`ERR gpio_args`; a driver failure returns `ERR gpio_io`.

- `Z`: high impedance, output and internal pull-up/pull-down disabled.
- `0`: push-pull output low.
- `1`: push-pull output high.

All advertised pins are initialized to Z once at firmware startup. Queries and
client connections do not change them. Settings survive retuning and client
disconnection, but reset to Z on reboot; they are not saved to flash. Responses
report the configured drive state, not the measured voltage at the pad.

Availability excludes input-only/nonexistent GPIOs, dedicated flash/PSRAM and
memory-supply pads (conservatively including optional memory), ESP-IDF-reserved
pins, native USB pins, and both UART pins when the UART transport is enabled.
Pin numbers are silicon GPIO numbers, not board connector labels. Firmware
cannot discover board wiring or which pads a particular module exposes.

GPIO commands obey the same serial ownership checks as receiver commands.
Clients must finish a burst, or stop and drain a spectrum stream, before sending
one. The browser provides a collapsed GPIO section at the bottom of the sidebar,
with a Z/0/1 button group for each available pin, and handles this sequencing.
