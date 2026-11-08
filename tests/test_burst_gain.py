from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class BurstGain(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_hardware_default_manual_selection_and_removed_software_command(self):
        header = Path(__file__).resolve().parents[1] / 'main/common/burst_gain.h'
        code = r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned forced, selected, mock_maximum=76;
static char response[128];
void esp_rom_delay_us(uint32_t us) { assert(us == 2); }
void force_rx_gain(unsigned force, unsigned gain, unsigned unused) { forced=force; selected=gain; }
static void reply(const char *s) { snprintf(response,sizeof(response),"%s",s); }
#define REG_READ(address) ((forced << 23)|(mock_maximum << 8))
#include "HEADER"
int main(void) {
 gain_apply(); assert(forced==0);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN HARDWARE -1"));
 assert(gain_command("GAIN MANUAL 23")); assert(forced==1 && selected==23);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN MANUAL 23"));
 assert(!gain_command("GAIN AUTO")); assert(forced==1 && selected==23);
 assert(!gain_command("GAIN MANUAL 99")); assert(selected==23);
 assert(gain_command("GAIN MANUAL 76")); assert(selected==76);
 assert(!gain_command("GAIN MANUAL 77")); assert(selected==76);
 mock_maximum=71; assert(!gain_command("GAIN MANUAL 72"));
 assert(gain_command("GAIN HARDWARE")); assert(forced==0);
 assert(gain_command("GAIN?")); assert(strstr(response,"GAIN HARDWARE -1"));
 return 0;
}
'''.replace('HEADER',str(header))
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'gain.c';source.write_text(code)
            for c5 in [0,1]:
                binary=Path(temp)/('gain-'+str(c5))
                subprocess.run(['cc','-std=c11',f'-DCONFIG_IDF_TARGET_ESP32C5={c5}',str(source),'-o',str(binary)],check=True)
                subprocess.run([str(binary)],check=True)

            # H2 uses a two-argument PHY API and a calibrated table bound in
            # phy_param. Releasing forced gain must leave RX enabled.
            h2 = code.replace('static unsigned forced, selected, mock_maximum=76;',
                'static unsigned forced, selected, receiver_on;\n'
                'uint8_t phy_param[84] = {[83]=76};\n'
                '#define mock_maximum phy_param[83]')
            h2 = h2.replace('unsigned gain, unsigned unused', 'unsigned gain')
            h2 = h2.replace('static void reply(',
                'void bt_rx_force(unsigned enabled) { receiver_on=enabled; }\nstatic void reply(')
            h2 = h2.replace('gain_apply(); assert(forced==0);',
                'gain_apply(); assert(forced==0 && receiver_on==1);')
            # The H2 table is only usable from index 2 to 56.
            h2 = h2.replace('assert(gain_command("GAIN MANUAL 76")); assert(selected==76);\n'
                            ' assert(!gain_command("GAIN MANUAL 77")); assert(selected==76);\n'
                            ' mock_maximum=71; assert(!gain_command("GAIN MANUAL 72"));',
                            'assert(gain_command("GAIN MANUAL 56")); assert(selected==56);\n'
                            ' assert(!gain_command("GAIN MANUAL 57")); assert(selected==56);\n'
                            ' assert(!gain_command("GAIN MANUAL 0") && !gain_command("GAIN MANUAL 1"));\n'
                            ' assert(gain_command("GAIN MANUAL 2")); assert(forced==1 && selected==2);')
            h2 = h2.replace('assert(gain_command("GAIN HARDWARE")); assert(forced==0);',
                'receiver_on=0; assert(gain_command("GAIN HARDWARE")); assert(forced==0 && receiver_on==1);')
            source.write_text(h2)
            binary=Path(temp)/'gain-h2'
            subprocess.run(['cc','-std=c11','-DCONFIG_IDF_TARGET_ESP32H2=1',str(source),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
