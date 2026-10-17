#!/usr/bin/env python3
"""Export one ESP-IDF build as a versioned artifact for esp-web-sdr (stdlib only)."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re


def firmware_build_date(data):
    # ESP-IDF esp_app_desc_t starts after the 24-byte image and 8-byte segment
    # headers. Its date field follows magic/security/reserved/version/name/time.
    if data[32:36] != bytes.fromhex('3254cdab'):
        return None
    try:
        return datetime.strptime(data[128:144].split(b'\0', 1)[0].decode('ascii'),
                                 '%b %d %Y').date().isoformat()
    except (ValueError, UnicodeDecodeError):
        return None


def firmware_version(data):
    marker = b'ESP-SDR-VERSION:'
    start = data.find(marker)
    if start < 0:
        return None
    start += len(marker)
    end = data.find(b'\0', start)
    try:
        if end < 0 or end - start > 512:
            raise ValueError('Invalid firmware version record')
        value = json.loads(data[start:end].decode('ascii'))
        stamp = value['build_timestamp']
        if not re.fullmatch(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z', stamp):
            raise ValueError('Invalid build timestamp')
        datetime.strptime(stamp, '%Y-%m-%dT%H:%M:%SZ')
        if value['build_date'] != stamp[:10] or not re.fullmatch(r'(?:[a-f0-9]{40}(?:-dirty)?|unknown)', value['revision']):
            raise ValueError('Invalid firmware metadata')
        if not re.fullmatch(r'esp32[a-z0-9]*(?:-[a-z0-9]+)?', value['profile']):
            raise ValueError('Invalid firmware profile')
        return value
    except (KeyError, TypeError, UnicodeDecodeError, ValueError) as exc:
        raise ValueError('Invalid embedded firmware version') from exc


def export(build, output, board, label, version, allow_larger_flash=False):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', board):
        raise ValueError('Board ID must contain only letters, numbers, dots, underscores or hyphens')
    cache = (build / 'CMakeCache.txt').read_text()
    for line in cache.splitlines():
        if re.match(r'(RING_PROBE|SAMPLE_RATE_PROBE|FILTER_REGISTER_PROBE|S3_RF_PROBE|S2_RF_PROBE|C5_TUNE_PROBE):', line):
            if line.rsplit('=', 1)[-1].upper() not in ('OFF', 'FALSE', '0', 'NO', ''):
                raise ValueError('Refusing to export diagnostic probe firmware')
    config = json.loads((build / 'config/sdkconfig.json').read_text())
    args = json.loads((build / 'flasher_args.json').read_text())
    target = config['IDF_TARGET']
    if args['extra_esptool_args']['chip'] != target:
        raise ValueError('Build target differs from flashing target')
    if not re.fullmatch(r'esp32[a-z0-9]*', target):
        raise ValueError('Expected an ESP32 target')
    settings = dict(args['flash_settings'])
    if settings['flash_size'] == 'detect':
        settings['flash_size'] = config.get('ESPTOOLPY_FLASHSIZE', '')
    size = re.fullmatch(r'(\d+)MB', settings['flash_size'])
    if not size:
        raise ValueError('Build must specify a concrete flash size in MB')
    limit = int(size[1]) * 1024 * 1024
    parts, payloads = [], []
    end = 0
    for i, (address, name) in enumerate(sorted(args['flash_files'].items(), key=lambda p: int(p[0], 0))):
        source = (build / name).resolve()
        if not source.is_relative_to(build.resolve()):
            raise ValueError('Flash image must be inside the build directory')
        data = source.read_bytes()
        offset = int(address, 0)
        if not data or offset < end or offset + len(data) > limit:
            raise ValueError('Empty, overlapping or out-of-flash image')
        end = offset + len(data)
        filename = f'{i}-{source.name}'
        parts.append(dict(name=filename, offset=offset, size=len(data),
                          sha256=hashlib.sha256(data).hexdigest(), md5=hashlib.md5(data).hexdigest()))
        payloads.append((filename, data))
    if not parts:
        raise ValueError('No firmware images in flasher_args.json')
    chip = 'ESP32' + ('-' + target[5:].upper() if target[5:] else '')
    manifest = dict(schema_version=1, version=version, variants={board: dict(
        revision=board, label=label, target=target, chip=chip, version=version,
        flash_size=settings['flash_size'], flash_size_policy='minimum' if allow_larger_flash else 'exact',
        flash_settings=settings, esptool_args=args['extra_esptool_args'], parts=parts)})
    if re.search(r'^ESP_SDR_STREAMING:BOOL=ON$', cache, re.MULTILINE):
        manifest['variants'][board]['application'] = 'soapysdr'
    elif re.search(r'^ESP_SDR_KORVO:BOOL=ON$', cache, re.MULTILINE):
        manifest['variants'][board]['application'] = 'display'
    if target == "esp32c2":
        manifest["variants"][board]["xtal_mhz"] = config["XTAL_FREQ"]
    dates = [date for _, data in payloads if (date := firmware_build_date(data))]
    if dates:
        manifest['variants'][board]['build_date'] = dates[-1]
    embedded = [record for _, data in payloads if (record := firmware_version(data))]
    if embedded:
        if len(embedded) != 1 or embedded[0]['profile'] != board:
            raise ValueError('Embedded firmware profile differs from export profile')
        record = embedded[0]
        manifest['variants'][board].update(
            git_revision=record['revision'], build_date=record['build_date'],
            build_timestamp=record['build_timestamp'])
    # Each matrix job writes a separate artifact directory; never merge in place.
    output.mkdir(parents=True, exist_ok=False)
    images = output / board
    images.mkdir()
    for name, data in payloads:
        (images / name).write_bytes(data)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    flash_args = [f"--flash-mode {settings['flash_mode']}",
                  f"--flash-freq {settings['flash_freq']}",
                  f"--flash-size {settings['flash_size']}"]
    flash_args += [f"{part['offset']:#x} {board}/{part['name']}" for part in parts]
    (output / 'flash_args').write_text('\n'.join(flash_args) + '\n')
    stub = '' if args['extra_esptool_args'].get('stub', True) else ' --no-stub'
    (output / 'flash_command.txt').write_text(
        f'python -m esptool --chip {target}{stub} --port PORT write-flash @flash_args\n')
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--board', required=True, help='Unique board/revision ID, e.g. esp32c5')
    parser.add_argument('--label', required=True, help='Human-readable board name')
    parser.add_argument('--version', required=True, help='Firmware source revision or release tag')
    parser.add_argument('--allow-larger-flash', action='store_true',
                        help='Allow this fixed flash layout on chips with larger flash')
    a = parser.parse_args()
    print(export(a.build_dir, a.output, a.board, a.label, a.version, a.allow_larger_flash))
