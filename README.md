# ESP-SDR firmware

<img src="docs/espargos-logo.png" width="40%" align="right" alt="ESPARGOS logo">

### Try it yourself: [Project overview](https://espargos.net/espsdr/) · [Browser SDR viewer](https://espargos.net/espsdr/app/) · [Browser firmware installer](https://espargos.net/espsdr/app/flash.html)

ESP-SDR turns the Wi-Fi/Bluetooth radio of an ordinary ESP32 board into a
receive-only software-defined radio. The firmware captures raw I/Q samples from
the chip's own receiver and sends them to your computer over USB or UART.
No extra SDR hardware is needed.

<br clear="all">

![ESP32 radio architecture: an undocumented debug path connects the ADC/DAC to the CPU, bypassing the fixed-function Wi-Fi modem.](docs/sdr-bypass.png)

The ESP32's receiver normally feeds its samples straight into the fixed-function
Wi-Fi modem. With the help of LLMs, we found an undocumented debug path that
lets the CPU read those samples instead.
[How it works](docs/how-it-works.md) explains the mechanism register by register. Parts of the firmware are
AI-generated: we understand the capture mechanism well on the ESP32-C61 used in
our ESPARGOS One array, but porting it to the whole ESP32 family by hand would
have been too much work.

## Quick start

1. Flash the firmware from the [browser installer](https://espargos.net/espsdr/app/flash.html).
2. Open the [browser viewer](https://espargos.net/espsdr/app/) and connect.

For continuous streaming into Gqrx, GNU Radio and friends, use an ESP32-S31
with [SoapyESPSDR](https://github.com/ESPARGOS/SoapyESPSDR), or an ESP32-S3 with
[esp-sdr-bridge](https://github.com/z2labs/esp-sdr-bridge).

## Supported chips

| Chip | Status | Native USB | UART0 TX / RX | Sample rates (MS/s) | Notes |
| --- | --- | --- | --- | --- | --- |
| ESP32 | ✅ | — | GPIO1 / GPIO3 | 80, 40, 16 | |
| ESP32-C2 | ✅ | — | GPIO20 / GPIO19 | 80, 40, 16 | 26 MHz crystal; rebuild for 40 MHz |
| ESP32-C3 | ✅ | Serial/JTAG | GPIO21 / GPIO20 | 80, 40, 16 | |
| ESP32-C5 | ✅ | Serial/JTAG | GPIO11 / GPIO12 | 80 … 4 | also receives 5 GHz |
| ESP32-C6 | ✅ | Serial/JTAG | GPIO16 / GPIO17 | 80, 40, 16 | continuous capture¹ (256 bins) |
| ESP32-C61 | ✅ | Serial/JTAG | GPIO11 / GPIO10 | 80 … 4 | continuous capture¹ (256 bins) |
| ESP32-H2 | ✅ | Serial/JTAG | GPIO24 / GPIO23 | 32, 16, 10.667, 6.4 | Bluetooth radio, see [esp32h2.md](docs/esp32h2.md) |
| ESP32-H21 | 🚧 | | | | |
| ESP32-H4 | 🚧 | | | | |
| ESP32-P4 | ❌ | | | | no radio |
| ESP32-S2 | ✅ | USB-OTG CDC | GPIO43 / GPIO44 | 80, 40, 16 | 4 MB flash |
| ESP32-S3 | ✅ | Serial/JTAG | GPIO43 / GPIO44 | 80, 40, 16 | continuous capture¹ and I/Q over USB |
| ESP32-S31 | ✅ | Serial/JTAG | GPIO58 / GPIO59 | 80 … 4 | continuous capture¹; streaming firmware |

✅ supported · 🚧 not yet supported · ❌ not possible

¹ **Continuous capture** means the chip records the signal without gaps while
it computes spectra, over native USB only. It can't FFT all of it: only a
small share of the signal, often a few percent or less, goes into a
spectrum. The viewer shows that share as FFT coverage. This mode exists at
256 FFT bins on the C6 and C61, and at all sizes on the S3 and S31. All other
spectra, including every spectrum over UART, come from separate snapshots
with gaps in between; the viewer then says "Reception has gaps between
snapshots".

All chips accept tuning from 100 to 6000 MHz, but the PLL only locks in a
limited range around 2.4 GHz (and 5 GHz on the C5). ESP32, C2, C3, S2 and S3
reach down to about 1.85 GHz using a 5/6 LO divider, the C6 to about 1.95 GHz;
see [rx-controls.md](docs/rx-controls.md).

Snapshots are bursts of up to 16,380 samples (8,190 on the C2 and 12,284 on the
S2), so there are gaps between them. Gain is not calibrated in dB.

## What you can do with it

**Burst I/Q captures** (all chips): signed 8-bit or packed 10-bit I/Q at the
rates above. This is what the browser viewer uses.

**On-chip spectra** (all chips): the firmware computes 256–2048-bin FFTs and
sends compact spectra instead of raw samples. On the C6, C61, S3 and S31 over
native USB, the receiver keeps recording between FFTs (continuous capture¹),
but only part of the signal is analyzed. See [spectrum.md](docs/spectrum.md). The original S3 Turbo
Mode was developed by Zoltan Doczi from [Z2Labs](https://www.z2labs.io/).

**Continuous I/Q on the ESP32-S3**: the second core filters and decimates the
16 MS/s stream to 15.625–250 kS/s, sent gap-free over native USB. See
[iq-stream.md](docs/iq-stream.md).

**ESP32-S31 streaming** (separate `esp32s31-stream` firmware, experimental):
continuous 8-bit I/Q at up to 20 MS/s over high-speed USB or 40 MS/s over
Gigabit Ethernet, for [SoapyESPSDR](https://github.com/ESPARGOS/SoapyESPSDR).
See [s31-streaming.md](docs/s31-streaming.md).

**Stand-alone displays** (optional builds): a live spectrum and waterfall on
an M5Stack Din Meter (ESP32-S3; the knob tunes, a press switches it to span
adjustment, 80 down to 2 MHz) or an ESP32-C3 SuperMini with a 0.42" OLED,
with no host computer. A USB host still gets the full protocol. See
[displays.md](docs/displays.md).

![Gqrx displaying an LTE signal at 2.63 GHz, continuously sampled at 40 MSa/s over Ethernet with an ESP32-S31 and SoapyESPSDR.](docs/gqrx-esp-sdr.png)

## Talking to the firmware

The firmware speaks a simple line-based protocol over native USB or UART
(2 Mbaud): `INFO`, `FREQ 2442`, `CAP16 4096 0` and so on, with binary payloads
protected by CRC32. The full command reference is in
[protocol.md](docs/protocol.md).

A few practical notes:

- **UART adapters** must handle 2 Mbaud reliably. Slow bridges (some CH340
  boards) can use the viewer's *Switch to 1 Mbaud*.
- **Power**: the radio draws current spikes when it starts. A board powered
  from a USB-UART adapter's 3.3 V pin can brown out and reboot in a loop; use a
  proper supply.
- **Linux**: ModemManager may probe Espressif USB devices and reset them. If
  the board resets unexpectedly, stop ModemManager or tell it to ignore the
  device.

## Build and flash

[firmware-targets.json](firmware-targets.json) lists every firmware variant
and the ESP-IDF commit it is pinned to. Check out that commit, run
`install.sh <target>` and source `export.sh`. Then build each chip in its own
build directory:

```sh
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=build-s3/sdkconfig \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash
```

The S31 needs `idf.py --preview`. `tools/build_firmware.py --profile <id>`
builds a release image the same way CI does. Host-side tests run with
`python3 -m unittest discover -s tests`. On real hardware,
`tools/check_spectrum.py` checks every spectrum profile of a connected board.

Chip-specific notes: [ESP32-C2](docs/esp32c2.md), [ESP32-H2](docs/esp32h2.md).

## Source layout

- `main/targets/<chip>/`: per-chip receiver, tuning and capture memory guard.
- `main/families/c5_c6_c61/`: receiver shared by the C5, C6 and C61.
- `main/targets/esp32s31/streaming/`: the S31 USB/Ethernet streaming application.
- `main/common/`: serial transport, gain, limits, bandwidth, spectrum and ring capture.
- `main/diagnostics/`: optional register probes, not part of release builds.
- `platform/esp32s2/`: patched copy of the ESP-IDF USB CDC console for the S2.

## Contributors

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/Jeija">
        <img src="https://github.com/Jeija.png?size=160" width="80" height="80" alt="Florian Euchner"><br>
        <b>Florian Euchner</b>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/zodoczi">
        <img src="https://github.com/zodoczi.png?size=160" width="80" height="80" alt="Zoltan Doczi"><br>
        <b>Zoltan Doczi</b>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/14sea">
        <img src="https://github.com/14sea.png?size=160" width="80" height="80" alt="14sea"><br>
        <b>14sea</b>
      </a>
    </td>
  </tr>
</table>

Thanks to [h0m3us3r](https://github.com/h0m3us3r) for
[eSpDR](https://github.com/h0m3us3r/eSpDR), whose 5/6 LO investigation informed
our lower-frequency tuning.

## License

ESP-SDR is licensed under the GNU General Public License, version 3 or later
(`GPL-3.0-or-later`); see [LICENSE](LICENSE). We chose the GPL because we want
improvements to flow back to the community: if you distribute a modified
version, you must make its source available under the GPL. Contributing
upstream is welcome but not required.

The capture mechanism itself is not covered by copyright, only this code is.
You are free to implement it independently under any license.

Third-party code keeps its own license, including the Apache-2.0 ESP-IDF code in
`platform/esp32s2/esp_usb_cdc_rom_console/`, the
[ESP-DSP component](components/esp-dsp/LICENSE), and the FFT kernels derived
from it in `main/targets/esp32s3/s3_fft_rnd.S` and
`main/targets/esp32s31/burst/s31_fft_rnd.S`.
