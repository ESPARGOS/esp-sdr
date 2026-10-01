# ESP-SDR firmware

<img src="docs/espargos-logo.png" width="40%" align="right" alt="ESPARGOS logo">

ESP-SDR turns the ESP32's built-in 2.4 GHz Wi-Fi radio into a
**software-defined radio (SDR)**. It lets you capture the radio signal itself
and process it in software, so you can view the spectrum and study signals
beyond ordinary Wi-Fi packets. No separate SDR hardware is needed. The
ESP32-C5 also supports reception in the 5 GHz band.

With the help of LLMs, we discovered an undocumented debug path that bypasses
the chip's fixed-function Wi-Fi modem. This gives software access to raw
radio samples, called **I/Q samples**, from the built-in receiver. ESP-SDR
currently captures short bursts of these samples and sends them to your
computer over USB or UART for analysis.

<br clear="all">

![ESP32 radio architecture: an undocumented debug path connects the ADC/DAC to the CPU, bypassing the fixed-function Wi-Fi modem.](docs/sdr-bypass.png)

The diagram shows the hardware's receive and transmit paths; this firmware
currently implements reception only.

[Project overview](https://espargos.net/espsdr/) ·
[Browser SDR viewer](https://espargos.net/espsdr/app/) ·
[Browser firmware installer](https://espargos.net/espsdr/app/flash.html)

**Parts of the firmware code are AI-generated.**
While we have a very good understanding of how the IQ sampling functionality works on the ESP32-C61 chip (used in our ESPARGOS One array), making IQ sampling work on the whole range of ESP32 family chips would have been too much work without LLM support.

## Chip support

| Chip | Status | Native USB | UART0 TX / RX | Minimum flash |
| --- | --- | --- | --- | --- |
| ESP32 | ✅ Supported | — | GPIO1 / GPIO3 | 2 MB |
| ESP32-C2 | 🚧 Unsupported | — | — | — |
| ESP32-C3 | ✅ Supported | Serial/JTAG | GPIO21 / GPIO20 | 2 MB |
| ESP32-C5 | ✅ Supported | Serial/JTAG | GPIO11 / GPIO12 | 2 MB |
| ESP32-C6 | ✅ Supported | Serial/JTAG | GPIO16 / GPIO17 | 2 MB |
| ESP32-C61 | ✅ Supported | Serial/JTAG | GPIO11 / GPIO10 | 2 MB |
| ESP32-H2 | 🚧 Unsupported | — | — | — |
| ESP32-H21 | 🚧 Unsupported | — | — | — |
| ESP32-H4 | 🚧 Unsupported | — | — | — |
| ESP32-P4 | ❌ Unsupported; no integrated radio | — | — | — |
| ESP32-S2 | ✅ Supported | USB-OTG CDC | GPIO43 / GPIO44 | 4 MB |
| ESP32-S3 | ✅ Supported | Serial/JTAG | GPIO43 / GPIO44 | 2 MB |
| ESP32-S31 | ✅ Supported | Serial/JTAG | GPIO58 / GPIO59 | 2 MB |

## ESP32-S3 Turbo Mode (SPEC)

On the ESP32-S3 the capture can also run **continuously**: the RF dump engine
fills a gapless ring over three SRAM banks, the S3 computes FFT spectra on chip
(256/1024/2048 bins) and streams only the spectra over the native USB port.
This gives ~1300 spectrum updates per second at 16 and 80 MS/s instead of ~23
bursts per second, and shrinks the blind gap between captures from ~42 ms to
under 150 us. Commands: `SPEC`, `RING`, `RINGCAP`, `DUAL`, `ASSIST`, `DC`
(capabilities `RING SPEC SPECN SPECSTAT DCT`); host tool: `tools/s3_ring.py`. The
browser viewer enables it automatically when the firmware reports `SPEC`.

The second core runs as a bare SPEC worker (ESP-IDF stays unicore), core 0
helps between bank switches, and unpacking uses the PIE SIMD unit: 2-5x more
FFT coverage than single-core, 1024/2048 bins also at 80 MS/s, and optional
live chip statistics in the stream. Details and benchmarks:
[docs/s3-turbo-dual-core.md](docs/s3-turbo-dual-core.md).

Turbo Mode developed by Zoltan Doczi from [https://www.z2labs.io](https://www.z2labs.io/)

## Commands and transport

Connect over native USB or a 3.3 V USB-to-UART adapter with crossed TX/RX
and common ground, using the pins above. Both interfaces carry the same
request/response protocol: send newline-terminated ASCII commands and read
text replies. Capture replies also include a binary I/Q payload.

Query `INFO` and `CAPS` to identify the firmware and supported features.
`LIMITS?` reports receive-control limits, `RANGE?` reports the tuning range,
and `TRANSPORT?` identifies the active interface. Configure reception with
`FREQ <MHz>`, `BANDWIDTH <MHz>` and `GAIN` commands.

Request a snapshot with `CAP16 <samples> <rate-index>` for signed 8-bit I/Q
or `CAP20 <samples> <rate-index>` for packed signed 10-bit I/Q. The reply is
`DATA <samples> <crc32-hex> <capture-microseconds>`, followed by exactly
`ceil(samples × bits-per-component × 2 / 8)` binary bytes. Verify the
payload CRC32 before using the samples. Rate indices 0–6 select 80, 40, 20, 10,
8, 4 or 16 MS/s respectively; use only rates advertised by `LIMITS?`.

Finish reading each reply before sending another command. Failures return
`ERR <reason>`. `SYNC <nonce>` echoes the nonce to let clients resynchronize
after an incomplete transfer. One client controls the radio at a time;
`RELEASE` or five seconds of idle time releases it, while other clients
receive `ERR busy`.

## Build and flash

[firmware-targets.json](firmware-targets.json) lists the supported profiles and
pins their ESP-IDF commits, including the preview SDK for S31. Check out the
matching SDK, initialize its submodules, run `install.sh <target>`, and source
`export.sh`.

Use a separate build directory and configuration for each chip:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=sdkconfig.s3 \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

Substitute the target and paths for your chip. S31 also requires `idf.py --preview`.

## Source layout

- `main/targets/<target>/`: chip receiver or adapter, tuning helpers, and the
  linker guard for its capture SRAM. CMake selects only the requested target.
- `main/families/c5_c6_c61/`: receiver shared by C5, C6, and C61; its `chip.h`
  comes from the selected target directory.
- `main/common/`: burst serial transport, gain control, limits, and bandwidth
  helpers. The gain-table wrapper is linked only for C61 and S31.
- `main/diagnostics/`: optional register probes, excluded from release exports.
- `platform/esp32s2/`: pinned ROM USB CDC compatibility component.

The application component and UART configuration stay in `main/`. Target SDK
defaults stay at the repository root for the build tools and ESP-IDF defaults
lookup. The firmware uses the burst protocol over UART/native USB; the former
Ethernet and vendor USB streaming application is no longer included.

Run `python3 -m unittest discover -s tests` for host checks. Build every profile
with `tools/build_firmware.py` and its pinned SDK before distributing a change;
the CI matrix does this automatically. Preserve the target SRAM guards and
gain-table linker wrappers when moving or refactoring receiver code.

## Receive controls

Hardware AGC is the default. `GAIN MANUAL <index>` sets manual gain;
`GAIN HARDWARE` restores AGC. `LIMITS?` reports available gain indices,
bandwidths, sample rates and bit depths. `BANDWIDTH <MHz>` sets approximate
analog bandwidth; zero selects the widest setting.

All eight chips accept tuning attempts from **100–6000 MHz in 1 MHz steps**.
The viewer shows an informational warning outside 2400–2483.5 MHz, with
5150–5895 MHz also treated as the supported 5 GHz Wi-Fi band on C5. The warning never blocks tuning.
These are software attempt limits; the expanded range has not been hardware
validated.

- **ESP32:** 80/40/16 MS/s.
- **C3:** 80 MS/s; 14–62 MHz analog bandwidth.
- **C5:** 11–48 MHz bandwidth; selects its 5 GHz RF path above 3000 MHz.
- **C61:** 80/40/20/10/8/4 MS/s; 13–54 MHz bandwidth.
- **C6:** 80 MS/s; 12–54 MHz bandwidth.
- **S2:** 80/40/16 MS/s; 15–60 MHz bandwidth; up to 12,284 complex samples.
- **S3:** 13–69 MHz bandwidth.
- **S31:** 80/40/20/10/8/4 MS/s; 13–54 MHz bandwidth.

Captures have gaps; nominal sample rates exceed sustained serial throughput.
Gain and power are uncalibrated. Extended tuning does not guarantee PLL lock
or reception; the viewer uses the ISM-band warning described above.

See [receive-control details](docs/rx-controls.md).
