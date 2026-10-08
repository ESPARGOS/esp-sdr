"""S31 capture setup must preserve the requested hardware gain mode."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class S31Gain(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_gain_survives_receiver_preparation(self):
        source = (Path(__file__).resolve().parents[1] /
                  'main/targets/esp32s31/burst/receiver.c').read_text()
        start = source.index('static void prepare_rx(void)')
        prepare = source[start:source.index('\n}', start) + 2]
        stub = r'''
#include <assert.h>
#include <stdbool.h>
static unsigned frequency_mhz = 2350;
static bool stale;
static bool rx_recalibration_stale(void) { return stale; }
static void s31_tune(unsigned mhz) { assert(mhz == frequency_mhz); stale = false; }
static bool hardware_agc = true;
static unsigned gain_code = 40, forced, selected, saturation, rx_clock;
/* The real PBUS setup releases manual gain, even on an already-running RX. */
static void phy_pbus_workmode(void) { forced = 0; saturation = 1; }
static void phy_pbus_xpd_tx_off(void) {}
static void phy_pbus_xpd_rx_on(unsigned on) {}
static void phy_set_rxclk_en(unsigned on) { rx_clock = on; }
static void phy_rfrx_sat_rst(unsigned on) { saturation = on; }
static void phy_force_rx_gain(unsigned force, unsigned gain) {
    assert(rx_clock); forced = force; selected = gain;
}
'''
        check = r'''
int main(void) {
    prepare_rx(); assert(!forced && saturation);
    hardware_agc = false;
    const unsigned gains[] = {0, 20, 40, 60, 71};
    for (unsigned i = 0; i < sizeof(gains)/sizeof(gains[0]); i++) {
        gain_code = gains[i];
        for (unsigned capture = 0; capture < 3; capture++) {
            stale = capture == 1;
            prepare_rx();
            assert(!stale);
            assert(forced && selected == gain_code && !saturation);
        }
    }
    hardware_agc = true;
    prepare_rx(); assert(!forced && saturation);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'gain.c'
            path.write_text(stub + prepare + check)
            binary = Path(tmp) / 'gain'
            subprocess.run(['cc', '-std=c11', str(path), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
