# ESP32-S31 Ethernet / high-speed USB receiver

**Experimental:** This streaming mode is under development. Signal quality,
including the remaining DC peak, still needs improvement.

The `esp32s31-stream` profile is a separate receive-only application for the
ESP32-S31 Function-CoreBoard with its YT8531 Gigabit Ethernet PHY and 16 MB
PSRAM. It is used with SoapyESPSDR, while the existing `esp32s31` profile
continues to serve the serial burst/FFT viewer.

This target uses preview ESP-IDF support and chip-specific diagnostic register
programming. Keep the pinned SDK revision: these internal PHY interfaces are
not a stable Espressif API. Hardware results are in the
[streaming validation report](testing/s31-streaming-2026-10-02.md).
The subsequent [signal-quality and usability report](testing/s31-quality-2026-10-02.md)
records hardware DC calibration, the branded web interface, and the IQ10 investigation.

## Build and connect

The streaming SDK revision and dependencies are pinned in
`firmware-targets.json`, `main/targets/esp32s31/streaming/idf_component.yml`, and
`dependencies.lock`. The first streaming configure downloads the pinned
Espressif components. The ordinary burst builds remain independent of them.

```sh
source /opt/esp32/esp-idf/export.sh
python tools/build_firmware.py --profile esp32s31-stream --version SOURCE_REVISION
```

For development:

```sh
idf.py --preview -B build-stream -DIDF_TARGET=esp32s31 \
  -DESP_SDR_STREAMING=ON -DSDKCONFIG=build-stream/sdkconfig \
  -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32s31-stream build
idf.py --preview -B build-stream -p /dev/ttyUSB0 -b 115200 flash
```

UART0 GPIO58/59 carries the console. The native high-speed USB connector
carries control and I/Q data; the Serial/JTAG connector is for flashing/debug.
The board uses DHCP on Ethernet. Its UART log prints the IPv4 address;
opening that address presents the receiver control page. Use the same address
in `driver=espsdr,host=ADDRESS`, or use `driver=espsdr,usb=1` for native USB.

## Capture and memory ownership

The live modem diagnostic bus reaches PARLIO through internal GPIO matrix
loopbacks. PARLIO captures signed I/Q into a circular GDMA ring in internal
RAM, independent of the TCM snapshot aperture. A core-1 task copies completed
DMA regions into a bounded internal-RAM packet ring; core 0 sends those packets
over Ethernet or USB. PSRAM carries code/constant data, not the hot sample path. Sample conversion happens on the host.

Capture uses eight DMA regions (about 32 KB) and 32 wire packets (44 KB).
Producer notifications wake the sender immediately. Ethernet resolves the HTTP
peer through lwIP's ARP cache, then sends normal IPv4/UDP frames through the
public MAC scatter/gather API, without per-packet socket allocation or private
DMA descriptor access. USB pipelines two 22 KB internal DMA buffers, and the
host keeps 32 bulk reads queued (2 MiB, about 50 ms at 20 MSa/s). These transports retain full signed 8-bit
I and Q; no IQ4 packing is used.

After copying a USB batch into its private DMA buffer, the sender immediately
releases those ring slots before waiting for the previous transfer. Thus both
the in-flight and staged USB batches are independent of the capture ring.
Partial packets are included when checking how many free ring slots a DMA
completion requires.

The sender and USB endpoint task run at priority 24 on core 0, ahead of the
Wi-Fi initialization/housekeeping task (23), Ethernet receive (22), TCP/IP
(20), and HTTP (18), and block when they have no work. USB completions must
outrank Wi-Fi: sharing priority 23 allows a 1 ms RTOS time slice to delay the
sender, enough to exhaust the small sample ring during 20 MSa/s reception.
USB JSON commands run on a separate core-0 worker at priority 18. Calibration
never blocks TinyUSB completion callbacks. Replies are submitted back on the
USB task with a session-generation check, so a bus reset discards stale
commands/replies. Core 1 handles capture at priority 22. Keeping control callbacks below the
sender matters even during Ethernet reception: otherwise a USB status probe
can hold up transmission long enough to exhaust the small internal ring.

The supported rates are 4, 8, 16, and 20 MSa/s on both transports, plus 40 MSa/s
on Ethernet. The USB firmware rejects 40 MSa/s because its 80 MB/s payload
exceeds USB 2.0 high-speed capacity.

At 20 MSa/s, USB carries 40 MB/s of I/Q plus packet headers; at 40 MSa/s,
Ethernet carries 80 MB/s of I/Q (about 687 Mbit/s including Ethernet framing,
preamble, and inter-frame gaps). A Gigabit link is required for the latter.

GPIO20–25, 33–34, 36–40, and 42–44 are reserved for the loopback lanes. Do not
attach other peripherals to them with this firmware. Ethernet uses RGMII
GPIO8–19, MDC/MDIO GPIO5/6, and PHY reset GPIO7.

The ISR records completion pointers, byte lengths, and absolute source sample
positions. It performs no packet conversion or network I/O. The packetizer
handles variable DMA completion lengths, retaining incomplete packet tails.
It checks for circular-buffer overwrite before publishing a packet. Buffer
exhaustion produces sample-position gaps and counters instead of blocking the
RF producer or silently overwriting queued output.

The TCM upper aperture is reserved because the diagnostic producer's initial
setup briefly enables the dump writer. Capture then runs through PARLIO. RF
setup is serialized on the capture task; all HTTP and USB mutations share a
control lock. Changes start a new epoch, so old queued samples cannot be
mistaken for newly tuned reception.

## RX protocol 2

USB uses VID:PID `303a:4531`, device revision `0200`, and one vendor interface
(class `ff`, subclass `53`, protocol `02`). Full-speed enumeration is possible,
but sample streaming requires high speed. Endpoints:

| Endpoint | Direction | Content |
| --- | --- | --- |
| `01` | Host to device | One JSON control request, fewer than 512 bytes |
| `81` | Device to host | One JSON response, fewer than 1024 bytes |
| `82` | Device to host | Consecutive I/Q packets |

Ethernet control uses `POST /api/rx` with the same JSON objects. Ethernet data
uses UDP with exactly one complete I/Q packet per datagram. A `start` request
supplies a UDP port (1024–65535); firmware uses the HTTP peer address as the
destination. It does not accept an arbitrary third-party destination address.

Control operations:

```json
{"op":"status"}
{"op":"configure","frequency":2442000000,"rate":16000000,"gain":40,"agc":0,"bandwidth":0}
{"op":"start","port":50000}
{"op":"stop"}
```

USB `start` omits `port`. Replies contain `protocol`, `epoch`, current receiver
settings, `gain_max`, active `transport`, `capture_drops`, `transport_drops`,
`capture_overruns`, `ring_drops`, `time_us`, `usb_connected`, `ethernet_mbps`, and `ip`. Errors contain an `error` string; HTTP
also returns 400. Unknown operations and unsupported setting values fail
without applying a partial configuration. Only one transport can own a stream.

Every I/Q packet is 1376 bytes, including a 32-byte little-endian header:

| Byte offset | Type | Meaning |
| --- | --- | --- |
| 0 | 4 bytes | ASCII `ESR2` |
| 4 | uint16 | Protocol version, 2 |
| 6 | uint16 | Signed bits per component, 8 |
| 8 | uint32 | Acquisition epoch |
| 12 | uint32 | Complex sample rate in Hz |
| 16 | uint64 | Absolute complex-sample position within the epoch |
| 24 | uint64 | Boot-relative microsecond time of first sample |
| 32 | 1344 bytes | 672 interleaved signed I, Q pairs |

USB transfer boundaries need not match packet boundaries. Hosts must retain
partial packets across transfers, including timeouts with partial data. Reject
invalid versions, formats, lengths, and rates. Source positions permit loss
accounting without a separate wrapping packet sequence. A new epoch starts
at sample position zero; intentional configuration changes are discontinuities,
not transport losses.

The clock anchor is sampled in software before DMA start; it is not a
calibrated RF timestamp. Lower acquisition rates use direct subsampling, not
filtered decimation. The 13 MHz minimum analog bandwidth cannot prevent
aliasing at the lower rates. Wider host output formats do not add ADC precision.

## Hardware AGC and frequency calibration

`agc` selects manual gain (0, the default) or hardware AGC (1) over USB or
Ethernet. `gain` retains the manual index for switching back. Update both the
firmware and SoapyESPSDR for Gqrx's hardware AGC checkbox to work; older
firmware is reported as not supporting AGC.

Each frequency change refreshes receive DC calibration at the selected
frequency while retaining startup I/Q correction. If the PHY replaces DC
calibration in the background, reception restarts with refreshed DC correction
and a new acquisition epoch. See [receive-control details](rx-controls.md).

## Analog DC correction and automatic bandwidth

`dc_correction` is 1 after boot. In manual gain mode, before starting an acquisition epoch,
`dc.c` measures raw I/Q while PARLIO is running. Each measurement allows
50 ms for sampling-clock/receiver settling, then averages at least 16,384
complex samples. Short measurements immediately after enabling capture
produced misleading DAC-response estimates on the bench S31.

The calibration adjusts only the two baseband DC DACs (blocks 2/3, bank 2).
It stores their codes in both modem gain tables and latches them through an
identical unused gain entry, keeping forced gain enabled. This follows the
hardware latching approach in `sensor-firmware`: receiver state transitions
can reload the corrected codes without freezing all PBUS receiver controls.
RF/baseband gain, RF DC and native I/Q calibration fields are preserved.
This additional single-gain DC adjustment is skipped during hardware AGC;
the fresh PHY calibration supplies the corrections for all gain steps.
The original PHY table entries are restored when reconfiguring reception.
The register mapping is corroborated by `phy_set_rx_gain_cal_dc_new` in the
pinned S31 PHY library. A measured 2×2 response matrix handles polarity and
cross-coupling.
Corrections are bounded to 32 codes per trial and 96 codes from the initial
point, with at most five iterations and three backtracking trials each.
Failed/poorly conditioned measurements restore the initial DAC codes;
unsuccessful correction trials restore the best measured codes. Calibration
is volatile and runs before the stream's timestamp/sample-count origin.

`dc_before_i/q`, `dc_after_i/q`, `dc_steps`, and `dc_j00/j01/j10/j11` describe
the current calibration in status replies. They are estimates, not a claim
that DC stays constant across time or every board. Set `dc_correction: 0`
to retain the PHY's original offset. Soapy exposes automatic DC correction
through its standard API and optionally removes residual drift on the host.

For this streaming profile, `bandwidth: 0` now selects the sample rate in
MHz, limited to the filter's 13 MHz minimum. `effective_bandwidth` reports
the selected value. Explicit 54 MHz retains the former widest response.
The ordinary burst profiles retain their original bandwidth semantics.

## Why this backend remains IQ8

The selected modem diagnostic bus exposes Q in bits 0–9 and I in bits 10–19.
The continuous route takes bits 2–9 and 12–19: exactly sixteen synchronous
GPIO/PARLIO lanes. The pinned SDK's `SOC_PARLIO_RX_UNIT_MAX_DATA_WIDTH` is 16
and `parlio_ll_rx_set_bus_width` accepts 1/2/4/8/16. There is one PARLIO instance
with one RX unit (`PARLIO_LL_INST_NUM` / `PARLIO_LL_RX_UNITS_PER_INST`), so a
second independent PARLIO receiver cannot supply the missing bits. Its S31 camera HAL also
sets `CAM_LL_DATA_WIDTH_MAX` to 16; RGB data stride is not parallel bus width.

True IQ10 would require twenty synchronous data bits and a new capture
backend (or a characterized hardware serializer). Two independent DMA
receivers would also need proven alignment, extra buffering, and more memory
bandwidth. The old TCM snapshot path provides wider words but does not meet
this target's continuous, gap-free streaming requirement. No IQ10 mode is
advertised and no zero-padded IQ8 data is labelled as IQ10.
