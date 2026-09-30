#!/usr/bin/env python3
"""Host side of the ESP32-S3 continuous modes (RING / RINGCAP / SPEC).

  python tools/s3_ring.py --port COM5 ring --ms 2000 --rate 6
  python tools/s3_ring.py --port COM5 --freq 2476 --gain 50 ringcap --rate 6 --out cw.npy
  python tools/s3_ring.py --port COM5 --freq 2476 --gain 50 spec --ms 60000 --stride 4 --out run1

`ringcap` checks gaplessness with a CW tone: the tone phase is fitted per
unit and must continue across the bank switches; any missing or duplicated
samples show up as a phase step of 2*pi*f*gap/fs.

`spec` writes <out>.npz (raw frames + metadata) and <out>_summary.csv
(one row per frame: t_s, f_peak_hz, p_peak_db, f_edge_hz, gain, ffts, flags).
Needs: pip install pyserial numpy
"""
import argparse, struct, sys, time, zlib

import numpy as np
import serial

FRAME_MAGIC = b"SPC1"
HEADER = struct.Struct("<4sIQIHBBHBB")  # 28 bytes, see s3_ring.c spec_header_t
REPORT_FIELDS = ("status detail units pairs elapsed_us late_max work_max "
                 "frames drops abandoned ffts stopped_by_host").split()
STATUS = ["OK", "ARG", "LATE", "AGE", "START", "END", "LENGTH", "TRANSPORT"]
RATE_HZ = {0: 80e6, 1: 40e6, 6: 16e6}


class Esp:
    def __init__(self, port):
        # DTR/RTS released before opening: USB-UART bridges with the usual
        # auto-reset circuit would otherwise reset the board on open.
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 2000000, 5
        self.s.dtr = self.s.rts = False
        self.s.open()
        time.sleep(0.1)
        self.s.reset_input_buffer()

    def line(self):
        raw = self.s.readline()
        if not raw:
            raise TimeoutError("no answer from the ESP32-S3")
        return raw.decode(errors="replace").strip()

    def cmd(self, text, expect_ok=True):
        self.s.write((text + "\n").encode())
        answer = self.line()
        if expect_ok and answer != "OK":
            raise RuntimeError(f"{text!r} -> {answer!r}")
        return answer

    def read_exact(self, n):
        data = self.s.read(n)
        if len(data) != n:
            raise TimeoutError(f"short read {len(data)}/{n}")
        return data


def parse_report(text, tag):
    parts = text.split()
    if not parts or parts[0] != tag:
        raise RuntimeError(f"unexpected reply: {text!r}")
    r = dict(zip(REPORT_FIELDS, map(int, parts[1:])))
    r["status_name"] = STATUS[r["status"]] if r["status"] < len(STATUS) else str(r["status"])
    return r


def print_report(r, rate_hz=None):
    print(f"status {r['status_name']} (detail {r['detail']}), units {r['units']}, "
          f"pairs {r['pairs']}, elapsed {r['elapsed_us'] / 1e3:.1f} ms")
    if rate_hz and r["elapsed_us"]:
        print(f"  pairs / (elapsed*fs) = {r['pairs'] / (r['elapsed_us'] * 1e-6 * rate_hz):.5f}"
              "  (≈1 when gapless; start/stop overhead makes it slightly <1)")
    print(f"  late_max {r['late_max']} pairs, work_max {r['work_max']} cycles "
          f"({r['work_max'] / 240:.1f} us)")
    if r["frames"] or r["ffts"]:
        print(f"  frames {r['frames']}, drops {r['drops']}, abandoned FFTs {r['abandoned']}, "
              f"FFTs {r['ffts']}, stopped_by_host {r['stopped_by_host']}")


def iq_from_words(words):
    w = words.astype(np.uint32)
    i = ((w << 22).astype(np.int32) >> 22).astype(np.float64)
    q = ((w << 12).astype(np.int32) >> 22).astype(np.float64)
    return i + 1j * q


# ---- commands --------------------------------------------------------------

def do_ring(esp, a):
    esp.s.write(f"RING {a.ms} {a.rate}\n".encode())
    if a.ms == 0:
        input("running until Enter... ")
        esp.s.write(b"\n")
    esp.s.timeout = a.ms / 1000 + 5
    r = parse_report(esp.line(), "RING")
    print_report(r, RATE_HZ[a.rate])
    return r["status"] == 0


def do_ringcap(esp, a):
    esp.s.write(f"RINGCAP {a.units} {a.rate}\n".encode())
    head = esp.line()
    if not head.startswith("RINGDATA"):
        print_report(parse_report(head, "RINGCAP"))
        return False
    _, units, rate_hz, n0, n1, n2, crc = head.split()
    counts = [int(n) for n in (n0, n1, n2)][: int(units)]
    raw = esp.read_exact(4 * sum(counts))
    if zlib.crc32(raw) != int(crc, 16):
        raise RuntimeError("RINGDATA CRC mismatch")
    r = parse_report(esp.line(), "RINGCAP")
    print_report(r)
    words = np.frombuffer(raw, "<u4")
    if a.out:
        np.save(a.out, words)
        print(f"saved {len(words)} words -> {a.out}")
    return check_gapless(words, counts, float(rate_hz))


def check_gapless(words, counts, fs):
    """CW phase continuity across unit boundaries."""
    z = iq_from_words(words)
    z -= z.mean()
    edges = np.cumsum([0] + counts)
    # Coarse tone frequency from unit 0, parabolic interpolation.
    seg = z[: counts[0]] * np.hanning(counts[0])
    nfft = 1 << (counts[0] - 1).bit_length() + 2
    spec = np.abs(np.fft.fft(seg, nfft))
    k = int(np.argmax(spec))
    lm, l0, lp = np.log(spec[(k - 1) % nfft]), np.log(spec[k]), np.log(spec[(k + 1) % nfft])
    k_f = k + 0.5 * (lm - lp) / (lm - 2 * l0 + lp)
    w = 2 * np.pi * (k_f if k_f < nfft / 2 else k_f - nfft) / nfft
    # Refine the frequency from phase slopes inside each unit only.
    slopes, weights = [], []
    for u in range(len(counts)):
        n = np.arange(edges[u], edges[u + 1])
        ph = np.unwrap(np.angle(z[n] * np.exp(-1j * w * n)))
        p = np.polyfit(n - n.mean(), ph, 1)
        slopes.append(p[0]); weights.append(len(n))
    w += np.average(slopes, weights=weights)
    f_bb = w * fs / (2 * np.pi)
    snr_db = 10 * np.log10(spec[k] ** 2 / np.median(spec ** 2))
    print(f"tone: {f_bb / 1e3:+.3f} kHz baseband, peak/median {snr_db:.1f} dB")
    if snr_db < 25:
        print("WARNING: weak tone; the phase test needs a clean CW well above the noise.")
    phases, rms = [], []
    for u in range(len(counts)):
        n = np.arange(edges[u], edges[u + 1])
        v = z[n] * np.exp(-1j * w * n)
        a0 = np.angle(v.sum())
        phases.append(a0)
        rms.append(np.degrees(np.std(np.angle(v * np.exp(-1j * a0)))))
    ok = True
    for u in range(len(counts) - 1):
        step = (phases[u + 1] - phases[u] + np.pi) % (2 * np.pi) - np.pi
        gap = step / w if w else float("nan")
        good = abs(gap) < 0.25
        ok &= good
        print(f"boundary {u}->{u + 1} at pair {edges[u + 1]}: phase step {np.degrees(step):+.2f} deg"
              f" -> {gap:+.3f} samples  {'OK' if good else 'GAP?'}")
    print(f"in-unit phase noise (rms): {', '.join(f'{x:.1f}' for x in rms)} deg")
    print("unambiguous for gaps up to ±%.1f samples at this tone" % (np.pi / abs(w)))
    print("GAPLESS" if ok else "NOT GAPLESS")
    return ok


def read_frame(esp, first4):
    rest = esp.read_exact(HEADER.size - 4 + 256 + 4)
    blob = first4 + rest
    body, crc = blob[:-4], struct.unpack("<I", blob[-4:])[0]
    if zlib.crc32(body) != crc:
        raise RuntimeError("SPEC frame CRC mismatch (stream out of sync)")
    h = HEADER.unpack(body[: HEADER.size])
    bins = np.frombuffer(body[HEADER.size:], np.uint8)
    return h, bins


def do_spec(esp, a):
    mode = 1 if a.mode == "max" else 0
    esp.s.write(f"SPEC {a.ms} {a.stride} {a.upf} {mode}\n".encode())
    head = esp.line()
    if not head.startswith("SPEC "):
        raise RuntimeError(f"SPEC refused: {head!r}")
    _, nfft, fs, threshold, lo_mhz = head.split()
    nfft, fs, lo = int(nfft), float(fs), float(lo_mhz) * 1e6
    print(f"SPEC running: nfft {nfft}, fs {fs / 1e6:g} Msps, LO {lo / 1e6:g} MHz"
          + ("" if a.ms else " (Ctrl+C to stop)"))
    esp.s.timeout = 2
    headers, spectra, t_last = [], [], time.time()
    stop_sent = False
    try:
        while True:
            try:
                first4 = esp.read_exact(4)
            except TimeoutError:
                if stop_sent:
                    raise
                continue
            if first4 == FRAME_MAGIC:
                h, bins = read_frame(esp, first4)
                headers.append(h[1:]); spectra.append(bins)
                if time.time() - t_last > 1:
                    t_last = time.time()
                    print(f"\r{len(spectra)} frames, t={h[2] / fs:.2f} s, drops {h[7]}   ", end="")
                continue
            text = (first4 + esp.s.readline()).decode(errors="replace").strip()
            r = parse_report(text, "SPECEND")
            break
    except KeyboardInterrupt:
        esp.s.write(b"\n")
        stop_sent = True
        esp.s.timeout = 5
        # Ctrl+C may have split a frame: resynchronise on the frame magic or
        # the report line, then drain until the report arrives.
        buf = b""
        while True:
            buf += esp.read_exact(1)
            if buf.endswith(FRAME_MAGIC):
                try:
                    h, bins = read_frame(esp, FRAME_MAGIC)
                    headers.append(h[1:]); spectra.append(bins)
                except RuntimeError:
                    pass
                buf = b""
            elif buf.endswith(b"SPECEND"):
                r = parse_report(("SPECEND" + esp.s.readline().decode(errors="replace")).strip(), "SPECEND")
                break
    print()
    print_report(r, fs)
    if not spectra:
        return False
    hdr = np.array(headers, dtype=np.float64)  # frame, pair_index, pairs, ffts, flags, gain, drops, log2n, step
    spec_db = np.stack(spectra).astype(np.float32) / hdr[0, 8]
    # FFT order -> centered; ESP convention is I+jQ = LO - RF, so RF grows to the left.
    spec_db = np.fft.fftshift(spec_db, axes=1)
    f_bb = (np.arange(nfft) - nfft // 2) * fs / nfft
    f_rf = lo - f_bb if a.invert else lo + f_bb
    order = np.argsort(f_rf)
    f_rf, spec_db = f_rf[order], spec_db[:, order]
    t = hdr[:, 1] / fs
    np.savez_compressed(a.out + ".npz", spectra_db=spec_db, f_hz=f_rf, t_s=t,
                        pairs=hdr[:, 2], ffts=hdr[:, 3], flags=hdr[:, 4], gain=hdr[:, 5],
                        frame=hdr[:, 0], fs=fs, lo_hz=lo, nfft=nfft, report=str(r))
    write_summary(a.out + "_summary.csv", t, f_rf, spec_db, hdr, a.edge_db)
    print(f"saved {len(t)} frames -> {a.out}.npz, {a.out}_summary.csv")
    return r["status"] == 0


def write_summary(path, t, f, s, hdr, edge_db):
    """Peak (parabolic) and upper edge (highest bin within edge_db of the
    frame's peak) per frame. DC bin +-1 is ignored for the peak."""
    df = f[1] - f[0]
    mid = len(f) // 2
    work = s.copy()
    work[:, mid - 1: mid + 2] = -np.inf
    k = np.argmax(work, axis=1)
    k = np.clip(k, 1, len(f) - 2)
    rows = np.arange(len(k))
    a, b, c = s[rows, k - 1], s[rows, k], s[rows, k + 1]
    den = a - 2 * b + c
    off = np.where(den != 0, 0.5 * (a - c) / np.where(den != 0, den, 1), 0)
    f_peak = f[k] + off * df
    p_peak = b - 0.25 * (a - c) * off
    above = s >= (p_peak[:, None] - edge_db)
    edge_idx = len(f) - 1 - np.argmax(above[:, ::-1], axis=1)
    with open(path, "w") as fh:
        fh.write("t_s,f_peak_hz,p_peak_db,f_edge_hz,gain,ffts,flags\n")
        for i in range(len(t)):
            fh.write(f"{t[i]:.6f},{f_peak[i]:.0f},{p_peak[i]:.2f},{f[edge_idx[i]]:.0f},"
                     f"{int(hdr[i, 5])},{int(hdr[i, 3])},{int(hdr[i, 4])}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--freq", type=int, help="LO in MHz (FREQ command)")
    ap.add_argument("--gain", type=int, help="manual gain index (GAIN MANUAL); omit for hardware AGC")
    ap.add_argument("--bandwidth", type=int, help="BANDWIDTH in MHz (0 = open)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("ring", help="bank rotation statistics, no data")
    p.add_argument("--ms", type=int, default=2000, help="0 = until Enter")
    p.add_argument("--rate", type=int, default=6, choices=(0, 1, 6))
    p = sub.add_parser("ringcap", help="3 gapless units + CW phase-continuity check")
    p.add_argument("--rate", type=int, default=6, choices=(0, 1, 6))
    p.add_argument("--units", type=int, default=3, choices=(1, 2, 3))
    p.add_argument("--out")
    p = sub.add_parser("spec", help="continuous 256-bin spectra, 16 Msps")
    p.add_argument("--ms", type=int, default=0, help="0 = until Ctrl+C")
    p.add_argument("--stride", type=int, default=4, help="FFT every n-th 256-pair block")
    p.add_argument("--upf", type=int, default=1, help="units (~0.77 ms each) per frame")
    p.add_argument("--mode", choices=("mean", "max"), default="max")
    p.add_argument("--edge-db", type=float, default=20.0, help="upper-edge threshold below peak")
    p.add_argument("--no-invert", dest="invert", action="store_false",
                   help="use RF = LO + f_bb (default assumes I+jQ = LO - RF)")
    p.add_argument("--out", default="spec_run")
    a = ap.parse_args()

    esp = Esp(a.port)
    # Resynchronise: stale bytes from port open or an aborted run may be
    # pending on either side. Read until the RELEASE acknowledgement.
    esp.s.timeout = 1
    for _ in range(8):  # the board may still be booting after a USB reset
        esp.s.write(b"\nRELEASE\n")
        try:
            if esp.line() == "OK":
                break
        except TimeoutError:
            continue
    else:
        sys.exit("no answer from the ESP32-S3")
    esp.s.timeout = 5
    time.sleep(0.05)
    esp.s.reset_input_buffer()
    caps = esp.cmd("CAPS", expect_ok=False)
    if "SPEC" not in caps.split():
        sys.exit(f"firmware lacks RING/SPEC: {caps}")
    info = esp.cmd("RINGINFO?", expect_ok=False).split()
    print(f"heap free {info[1]} B (min {info[2]} B), {info[3]} banks, threshold {info[4]} pairs")
    if a.freq:
        esp.cmd(f"FREQ {a.freq}")
    if a.gain is not None:
        esp.cmd(f"GAIN MANUAL {a.gain}")
    if a.bandwidth is not None:
        esp.cmd(f"BANDWIDTH {a.bandwidth}")
    ok = {"ring": do_ring, "ringcap": do_ringcap, "spec": do_spec}[a.cmd](esp, a)
    esp.s.write(b"RELEASE\n")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
