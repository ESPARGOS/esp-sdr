# ESP32-S31 Ethernet / high-speed USB receiver

**Experimental.** Signal quality, especially the DC peak, still needs work.

The `esp32s31-stream` firmware turns an ESP32-S31 Function-CoreBoard (YT8531
Gigabit Ethernet PHY, 16 MB PSRAM) into a continuous, receive-only I/Q source
for [SoapyESPSDR](https://github.com/ESPARGOS/SoapyESPSDR):

| Transport | Sample rates | Format |
| --- | --- | --- |
| Gigabit Ethernet (UDP) | 4, 8, 16, 20, 40 MSa/s | 8-bit I + 8-bit Q |
| High-speed USB | 4, 8, 16, 20 MSa/s | 8-bit I + 8-bit Q |

The ordinary `esp32s31` firmware is the one for the browser viewer; this one
does not speak the serial burst protocol.

## Build, flash, connect

The streaming firmware relies on internal PHY interfaces that are not a stable
Espressif API, so use the ESP-IDF commit pinned in `firmware-targets.json`.

```sh
idf.py --preview -B build-stream -DIDF_TARGET=esp32s31 \
  -DESP_SDR_STREAMING=ON -DSDKCONFIG=build-stream/sdkconfig \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s31-stream build
idf.py --preview -B build-stream -p /dev/ttyUSB0 -b 115200 flash
```

Flash over the UART connector: this firmware takes over the USB PHY, so the
USB Serial/JTAG port disappears while it runs. UART0 (GPIO58/59) also carries
the log, which prints the board's DHCP address. Open that address in a browser
for the receiver control page, and use it in SoapyESPSDR as
`driver=espsdr,host=ADDRESS`. For USB, connect the native high-speed USB
connector and use `driver=espsdr,usb=1`.

40 MSa/s needs Gigabit Ethernet all the way (about 687 Mbit/s on the wire) and
a host whose UDP receive buffer is large enough. On Linux, raise
`net.core.rmem_max` to at least 32 MB, or the kernel drops packets before
SoapyESPSDR sees them. USB tops out at 20 MSa/s (40 MB/s of samples).

## Control protocol

Ethernet: `POST /api/rx` with a JSON object. USB: VID:PID `303a:4531`, one
vendor interface (class `ff`, subclass `53`, protocol `02`) with three bulk
endpoints: `01` takes one JSON request (< 512 bytes), `81` returns one JSON
reply (< 1024 bytes), `82` streams I/Q packets. USB streaming needs a high-speed
link.

```json
{"op":"status"}
{"op":"configure","frequency":2442000000,"rate":16000000,"gain":40,"agc":0,"bandwidth":0}
{"op":"start","port":50000}
{"op":"stop"}
```

- `configure` may contain any subset of the settings. Invalid values fail the
  whole request (HTTP 400 with an `error` string); nothing is partially applied.
- `start` over Ethernet names a UDP port; packets go to the address that sent
  the HTTP request. Over USB, `start` has no port.
- Only one transport can own the stream at a time.
- `agc`: 0 = manual gain (`gain` is a PHY table index), 1 = hardware AGC.
- `bandwidth`: analog filter in MHz, 13–54; 0 picks the sample rate
  (minimum 13). `effective_bandwidth` reports the result.
- `dc_correction`: 1 (default) runs an analog DC calibration whenever reception
  starts with manual gain; 0 keeps the PHY's own offset.

Replies always contain the current settings plus `protocol`, `epoch`,
`transport`, `gain_max`, the drop counters (`capture_drops`, `transport_drops`,
`capture_overruns`, `ring_drops`), `time_us`, `usb_connected`,
`ethernet_mbps`, `ip`, and the DC calibration results.

The control interface is unauthenticated: only use it on a trusted network.

### I/Q packets

Every packet is 1376 bytes: a 32-byte little-endian header and 672 I/Q pairs.
Over Ethernet, each UDP datagram holds exactly one packet. Over USB, packets
are concatenated, and transfer boundaries need not match packet boundaries.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | 4 bytes | `ESR2` |
| 4 | uint16 | Protocol version, 2 |
| 6 | uint16 | Bits per component, 8 |
| 8 | uint32 | Acquisition epoch |
| 12 | uint32 | Sample rate in Hz |
| 16 | uint64 | Position of the first sample within the epoch |
| 24 | uint64 | Microseconds since boot at the first sample |
| 32 | 1344 bytes | 672 signed I, Q pairs |

Samples are `(I, Q)` pairs, ready to use as `I + jQ`. The second component
is not negated as it is in [burst captures](protocol.md#captures) and the
S3 continuous stream.

Each configuration change starts a new epoch, with positions counting from
zero again. Within an epoch, a jump in position is lost data. The timestamp
comes from a software clock read just before DMA starts; it is not a
calibrated RF timestamp.

## How it works

The receiver's internal diagnostic bus carries 10-bit I and Q. The upper eight
bits of each are routed through the GPIO matrix (GPIO 20–25, 33–34, 36–40 and
42–44, which must stay unconnected) and back into the PARLIO parallel input,
16 lanes wide. PARLIO writes them into a circular DMA ring in internal RAM.
There is no CPU in the sample path until packetization:

1. The DMA interrupt records each region and its absolute sample position.
   On S31, the newest reported region can still be receiving DMA writes.
2. A task on core 1 waits for the following region to be reported before
   copying that data into a ring of 32 wire packets, starting a new packet
   header every 672 samples. The DC calibration uses the same rule.
3. A task on core 0 sends finished packets as UDP frames through the Ethernet
   MAC, or as batches of two 22 KB USB transfers.

If the network or USB can't keep up, packets are dropped and counted, and the
sample positions show exactly where. The producer never blocks and never
overwrites queued output. If capture falls behind, it resumes at the latest
region safe to read; data overwritten during a copy is discarded before any
packets from that copy are published. Stopping also resets the unfinished
PARLIO frame so the next acquisition starts with a full frame length.
Task priorities matter here: the USB and sender tasks run above Wi-Fi
housekeeping, and control requests run below them, so a status request can't
stall the stream.

Why only 8 bits: PARLIO on the S31 has a single 16-bit receive unit, so 2 × 10
bits don't fit. Full 10-bit samples would need a different capture path, so
the firmware doesn't advertise a 10-bit mode at all.

Lower sample rates simply take every n-th sample. There is no digital
decimation filter, and the 13 MHz minimum analog bandwidth can't prevent
aliasing at 4 or 8 MSa/s.

### DC correction

Each tuning refreshes the PHY's receive DC calibration. In manual-gain mode,
the firmware then measures the remaining offset (after 50 ms of settling,
averaging at least 16,384 samples) and adjusts the two baseband DC DACs. It
uses a measured 2×2 response matrix, so polarity and cross-coupling don't
matter, and takes at most five steps with bounded corrections. A failed
measurement restores the previous codes. Results are in the status reply
(`dc_before_*`, `dc_after_*`, `dc_steps`, `dc_j*`). AGC mode relies on the
PHY's own calibration for every gain step instead. SoapyESPSDR can also remove
residual DC on the host.
