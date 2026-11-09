# Stand-alone displays

Two optional builds draw a live spectrum and waterfall on the board itself, with
no host computer. While a host talks to the receiver over USB (ESP-WebSDR,
esp-sdr-bridge, scripts), the display pauses and every protocol command behaves
as in the standard firmware. The display resumes when the host's five-second
serial lease expires.

Both features are off by default and are enabled with an extra defaults file.

## M5Stack Din Meter (ESP32-S3)

240×135 colour LCD (ST7789), rotary encoder with push button.

Each display line is a 45 ms run of the continuous S3 capture (both cores,
gapless RF writer). The on-chip FFT frames that `SPEC` would send over USB go
to the display through `ring_capture_local_sink` instead. The LCD is drawn
between runs, while interrupts are enabled.

```
2437.000 MHZ  PK          SPAN 80M   centre frequency, detector, span; the knob's target is highlighted
2397.0   PK 2462.13 +23DB   2477.0   edge frequencies, strongest bin (dB above the noise floor)
|    |    |    |    |    |    |       ticks every tenth of the span
live spectrum                        50 dB tall, grid every 10 dB
waterfall                            40 dB colour range above the tracked floor, newest line on top
```

| Control | Action |
| --- | --- |
| Turn | Tune (TUNE) or change span (SPAN, clockwise zooms in) |
| Press | Switch the knob between TUNE and SPAN |
| Long press (0.7 s) | Peak hold ↔ average detector |

Tuning covers 2210–2700 MHz in 1 kHz steps, using the PLL offset that `FOFS`
also uses. The step per detent follows the span, and fast turning moves five
steps per detent. A host `FREQ` command clears an offset set with the knob.

| Span | Sample rate | FFT | Bin width | Analog filter |
| --- | --- | --- | --- | --- |
| 80, 60 MHz | 80 MS/s | 1024 | 78.1 kHz | 69 / 62 MHz |
| 40 MHz | 40 MS/s | 1024 | 39.1 kHz | 40 MHz |
| 20 MHz | 40 MS/s | 2048 | 19.5 kHz | 24 MHz |
| 16 MHz | 16 MS/s | 1024 | 15.6 kHz | 14 MHz |
| 10, 5, 2 MHz | 16 MS/s | 2048 | 7.8 kHz | 13 MHz |

Each pixel shows the strongest bin it covers. Bins next to 0 Hz are bridged
from their neighbours to hide the zero-IF DC/LO leakage.

Board notes:

- GPIO46 (battery power hold) is driven high at boot.
- The build disables UART0, which is not wired out, and keeps the SPI ISR in
  flash. Both are needed to fit the S3's SRAM next to the RF ring.
- The UI's ~22 KiB of display buffers are released while a host holds the
  lease. Host modes that allocate at run time, such as `IQS`, keep their memory.
- LCD, encoder, button, buzzer, RTC and power-hold pins are removed from
  browser GPIO control.

```bash
idf.py -B build-dinmeter -D SDKCONFIG=build-dinmeter/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.dinmeter" set-target esp32s3 build
idf.py -B build-dinmeter -p PORT flash
```

`menuconfig` → *ESP-SDR transport* has a 180° LCD flip, encoder reversal and
counts per detent (default 4).

## ESP32-C3 SuperMini with 0.42" OLED

72×40 monochrome OLED (SSD1306, I2C SDA GPIO5 / SCL GPIO6, address 0x3C) and the
BOOT button (GPIO9).

The C3 has only snapshot capture. Each 202 µs, 80 MS/s snapshot is first scanned
for block energy. Only its three strongest 256-sample blocks, plus one rotating
block for the floor, are transformed and max-held into the line. Short Wi-Fi
and Bluetooth bursts are therefore caught far more often than by transforming
consecutive blocks. The 67.5 MHz display span uses three 312.5 kHz bins per
column. Levels are relative to a tracked noise floor and ordered-dithered to
one bit.

| Control | Action |
| --- | --- |
| Short press | Next centre frequency: 2437 → 2462 → 2480 → 2412 → 2400 MHz |
| Long press (0.6 s) | View: spectrum + waterfall → waterfall → slow waterfall (2 lines/s) |

```bash
idf.py -B build-c3oled -D SDKCONFIG=build-c3oled/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.c3oled" set-target esp32c3 build
idf.py -B build-c3oled -p PORT flash
```

`menuconfig` has the I2C pins, button pin, first visible column (default 28) and
a 180° flip. If no panel answers on I2C, the feature stays idle.

## Serial commands

| Command | Reply |
| --- | --- |
| `DM?` | Din Meter status: span, knob mode, detector, line/frame counts, last run status, floor, peak, heap, encoder count |
| `DMLINE` | Din Meter: noise floor and the latest 240 column levels (0.5 dB codes) |
| `DMWF` | Din Meter: the 65×240 waterfall as hex palette levels, newest line first. Passive: it neither takes the serial lease nor pauses the display |
| `OLED?` | C3: panel presence, on/off, view, floor, line count, FFTs in the last line |
| `OLED ON` / `OLED OFF`, `OLEDMODE 0..2` | C3: blank the panel, select the view |
| `OLEDDUMP` | C3: the frame buffer as 40 lines of `#`/`.` |
| `OLEDLINE` | C3: noise floor and the latest 72 column levels |

`CAPS` adds `DINMETER` or `OLED` when the feature is built in.
