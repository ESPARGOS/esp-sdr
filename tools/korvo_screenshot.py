#!/usr/bin/env python3
"""Save the ESP32-S31-Korvo-1 LCD frame buffer as PNG over the console UART."""
import argparse
import sys
import time

import serial
from PIL import Image


def capture(port, baud, commands=(), settle=0.8, timeout=15):
    with serial.Serial(port, baud, timeout=1) as s:
        for command in commands:
            s.write(command.encode() + b'\n')
            time.sleep(settle)
        s.reset_input_buffer()
        s.write(b'SHOT\n')
        deadline = time.time() + timeout
        header = b''
        while time.time() < deadline:
            line = s.readline()
            if line.startswith(b'SHOT '):
                header = line
                break
        if not header:
            raise RuntimeError('No frame header from the device')
        width, height = (int(v) for v in header.split()[1:3])
        size = width * height * 2
        data = bytearray()
        while len(data) < size and time.time() < deadline:
            chunk = s.read(size - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != size:
            raise RuntimeError(f'Short frame: {len(data)} of {size} bytes')
    pixels = memoryview(bytes(data)).cast('H')
    rgb = bytearray(width * height * 3)
    for i, v in enumerate(pixels):
        rgb[3 * i] = ((v >> 11) & 31) * 255 // 31
        rgb[3 * i + 1] = ((v >> 5) & 63) * 255 // 63
        rgb[3 * i + 2] = (v & 31) * 255 // 31
    return Image.frombytes('RGB', (width, height), bytes(rgb))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--port', default='/dev/ttyUSB1')
    parser.add_argument('--baud', type=int, default=2000000)
    parser.add_argument('--command', action='append', default=[], metavar='LINE',
                        help='console command to send before the capture, e.g. "SHEET 1"')
    parser.add_argument('--settle', type=float, default=0.8, help='seconds to wait after each command')
    args = parser.parse_args()
    try:
        capture(args.port, args.baud, args.command, args.settle).save(args.output)
    except (RuntimeError, serial.SerialException) as exc:
        sys.exit(str(exc))
    print(args.output)
