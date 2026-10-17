#!/usr/bin/env python3
"""Build/export a catalog profile, or print the GitHub Actions build matrix."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from export_web_firmware import export

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', action='store_true')
    parser.add_argument('--profile')
    parser.add_argument('--version', help='Source commit or release tag, recorded in the artifact')
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts')
    parser.add_argument('--build-root', type=Path, default=ROOT, help='Parent of per-profile build directories')
    parser.add_argument('--jobs', type=int, default=2, help='Parallel compiler jobs (default: 2)')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    catalog = json.loads((ROOT / 'firmware-targets.json').read_text())
    if args.matrix:
        print(json.dumps({'include': catalog['profiles']}, separators=(',', ':')))
        return
    profiles = {p['id']: p for p in catalog['profiles']}
    if args.profile not in profiles or not args.version:
        parser.error('Specify --profile from firmware-targets.json and --version')
    profile = profiles[args.profile]
    sdk = os.environ.get('IDF_PATH')
    if not sdk:
        parser.error('Activate the pinned ESP-IDF environment first')
    revision = subprocess.check_output(['git', '-C', sdk, 'rev-parse', 'HEAD'], text=True).strip()
    if revision != profile['idf_ref']:
        parser.error(f"Profile requires ESP-IDF {profile['idf_ref']}; found {revision}")
    build = args.build_root.resolve() / ('build-' + args.profile)
    output = args.output.resolve() / args.profile
    if output.exists():
        parser.error(f'Output already exists: {output}; choose a fresh artifact directory')
    registry = profile.get('streaming') or profile.get('korvo')
    env = dict(os.environ, IDF_PY_BUILD_JOBS=str(args.jobs), IDF_COMPONENT_MANAGER='1' if registry else '0')
    command = [sys.executable, str(Path(sdk) / 'tools/idf.py')]
    if profile['preview']:
        command.append('--preview')
    command += ['-C', str(ROOT), '-B', str(build), '-DIDF_TARGET=' + profile['target'],
                '-DSDKCONFIG=' + str(build / 'sdkconfig'),
                '-DSDKCONFIG_DEFAULTS=' + str(ROOT / profile.get('sdkconfig', 'sdkconfig.defaults.' + profile['target'])),
                '-DESP_SDR_STREAMING=' + ('ON' if profile.get('streaming') else 'OFF'),
                '-DESP_SDR_KORVO=' + ('ON' if profile.get('korvo') else 'OFF')]
    if not registry:
        command += ['-DRING_PROBE=OFF', '-DSAMPLE_RATE_PROBE=OFF', '-DFILTER_REGISTER_PROBE=OFF',
                    '-DS3_RF_PROBE=OFF', '-DC5_TUNE_PROBE=OFF', '-DS2_RF_PROBE=OFF']
    command.append('build')
    subprocess.run(command, env=env, check=True)
    export(build, output, profile['id'], profile['label'], args.version, profile['allow_larger_flash'])
    provenance = {'profile': profile['id'], 'version': args.version, 'idf_commit': revision}
    (output / 'build-info.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(output)


if __name__ == '__main__':
    main()
