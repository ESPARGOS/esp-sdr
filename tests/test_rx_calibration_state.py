"""Exercise generation tracking and the pinned DC wrapper calling conventions."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

MAIN = Path(__file__).resolve().parents[1] / 'main'


@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class CalibrationState(unittest.TestCase):
    def test_measurement_generation_and_argument_forwarding(self):
        targets = {
            'ESP32C5': ('phy_set_rx_gain_cal_dc', 4),
            'ESP32C61': ('phy_set_rx_gain_cal_dc_new', 4),
            'ESP32S31': ('phy_set_rx_gain_cal_dc_new', 4),
            'ESP32C6': ('set_rx_gain_cal_dc_new', 5),
            'ESP32C2': ('ram_set_rx_gain_cal_dc', 2),
            'ESP32H2': ('set_rx_gain_cal_dc', 1),
            **{c: ('set_rx_gain_cal_dc', 10)
               for c in ('ESP32S3', 'ESP32C3')},
        }
        for chip, (symbol, count) in targets.items():
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as tmp:
                p = Path(tmp)
                (p/'esp_private').mkdir()
                (p/'sys').mkdir()
                (p/'sdkconfig.h').write_text('')
                (p/'sys/lock.h').write_text(
                    '#pragma once\ntypedef void *_lock_t;\n'
                    'void _lock_acquire(_lock_t *);\nvoid _lock_release(_lock_t *);\n')
                (p/'esp_private/phy.h').write_text(
                    '#include <sys/lock.h>\n_lock_t phy_get_lock(void);\n')
                args = ', '.join(f'uintptr_t a{i}' for i in range(count))
                values = ', '.join(str(0x12340+i) for i in range(count))
                checks = '\n'.join(f'assert(a{i} == {0x12340+i});' for i in range(count))
                (p/'vendor.c').write_text(f'''
#include <assert.h>
#include <stdint.h>
#include <sys/lock.h>
#include "rx_recalibration.h"
static int locked, calls, measuring;
_lock_t phy_get_lock(void) {{ return &locked; }}
void _lock_acquire(_lock_t *p) {{ assert(*p == &locked && !locked); locked = 1; }}
void _lock_release(_lock_t *p) {{ assert(*p == &locked && locked); locked = 0; }}
extern void __wrap_{symbol}({args});
void __real_{symbol}({args}) {{
    {checks}
    if (measuring) assert(locked);
    ++calls;
}}
void rx_recalibrate_locked(unsigned mhz) {{
    assert(locked && mhz == 2350);
    measuring = 1;
    __wrap_{symbol}({values});
    __wrap_{symbol}({values});
    measuring = 0;
}}
int main(void) {{
    assert(rx_recalibration_stale());
    rx_recalibrate(2350);
    assert(!locked && !rx_recalibration_stale() && calls == 2);
    /* No temperature-stamp change, but a reference DC measurement occurred. */
    __wrap_{symbol}({values});
    assert(rx_recalibration_stale());
    rx_recalibrate(2350);
    assert(!rx_recalibration_stale() && calls == 5);
    __wrap_{symbol}({values});
    __wrap_{symbol}({values});
    assert(rx_recalibration_stale());
    rx_recalibrate(2350);
    assert(!rx_recalibration_stale() && calls == 9);
}}
''')
                subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                '-DCONFIG_IDF_TARGET_'+chip+'=1', '-I'+tmp,
                                '-I'+str(MAIN/'common'),
                                str(MAIN/'common/rx_calibration_state.c'),
                                str(p/'vendor.c'), '-o', str(p/'check')], check=True)
                subprocess.run([str(p/'check')], check=True)

    def test_legacy_dc_table_changes_under_phy_lock(self):
        for chip, bb, rf, chan in [('ESP32', 16, 136, 0), ('ESP32S2', 60, 48, 56)]:
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as tmp:
                p = Path(tmp)
                (p/'sys').mkdir()
                (p/'esp_private').mkdir()
                (p/'sdkconfig.h').write_text('')
                (p/'sys/lock.h').write_text(
                    '#pragma once\ntypedef void *_lock_t;\n'
                    'void _lock_acquire(_lock_t *);\nvoid _lock_release(_lock_t *);\n')
                (p/'esp_private/phy.h').write_text(
                    '#include <sys/lock.h>\n_lock_t phy_get_lock(void);\n')
                (p/'vendor.c').write_text(f'''
#include <assert.h>
#include <string.h>
#include <sys/lock.h>
#include "rx_recalibration.h"
unsigned char phy_rxbb_dc[{bb}], phy_rxrf_dc[{rf}], phy_chan_dc[56];
static int locked, acquisitions;
_lock_t phy_get_lock(void) {{ return &locked; }}
void _lock_acquire(_lock_t *p) {{ assert(*p == &locked && !locked); locked = 1; ++acquisitions; }}
void _lock_release(_lock_t *p) {{ assert(*p == &locked && locked); locked = 0; }}
void rx_recalibrate_locked(unsigned mhz) {{
    assert(locked && mhz == 2350);
    memset(phy_rxbb_dc, 42, sizeof phy_rxbb_dc);
    memset(phy_rxrf_dc, 43, sizeof phy_rxrf_dc);
    memset(phy_chan_dc, 44, sizeof phy_chan_dc);
}}
int main(void) {{
    assert(rx_recalibration_stale());
    rx_recalibrate(2350);
    assert(!rx_recalibration_stale());
    unsigned char *tables[] = {{phy_rxbb_dc, phy_rxrf_dc, phy_chan_dc}};
    unsigned sizes[] = {{{bb}, {rf}, {chan}}};
    for (unsigned t = 0; t < 3; ++t) {{
        for (unsigned i = 0; i < sizes[t]; ++i) {{
            tables[t][i] ^= 1;
            assert(rx_recalibration_stale());
            rx_recalibrate(2350);
            assert(!rx_recalibration_stale());
        }}
    }}
    assert(!locked && acquisitions > 100);
}}
''')
                subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                '-DCONFIG_IDF_TARGET_'+chip+'=1', '-I'+tmp,
                                '-I'+str(MAIN/'common'),
                                str(MAIN/'common/rx_calibration_state.c'),
                                str(p/'vendor.c'), '-o', str(p/'check')], check=True)
                subprocess.run([str(p/'check')], check=True)
