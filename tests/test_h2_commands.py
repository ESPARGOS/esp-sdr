"""Exercise the H2 command parser and negotiated limits with hardware stubs."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class H2Commands(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_commands(self):
        root = Path(__file__).resolve().parents[1]
        source = (root / 'main/targets/esp32h2/receiver.c').read_text()
        frequency = source[source.index('#define H2_FREQ_MIN'):source.index('#define send_bytes')]
        handler = source[source.index('static void handle_command('):source.index('void app_main(')]
        stub = r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include "rx_tuning.h"
#include <stdbool.h>
static bool burst_version_command(const char *s) { return false; }
static bool burst_gpio_command(const char *s) { return false; }
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#define BURST_SERIAL_UART 1
#define IQ_WORDS 16380u
#define CONFIG_IDF_TARGET_ESP32H2 1
#define CONFIG_ESP_SDR_UART_ENABLED 1
static unsigned frequency_mhz, captures, last_samples, last_format;
static bool rx_ready;
static int rx_filter=-1;
static unsigned chip_i2c_readReg(unsigned b,unsigned h,unsigned r){return 45;}
static unsigned calibrated_mhz,pll_mhz,pll_writes;
static void set_chanfreq(unsigned mhz,unsigned mode) { calibrated_mhz=mhz; }
static void phy_set_freq(unsigned mhz,int offset) { pll_mhz=mhz;pll_writes++; }
static char response[256];
static int burst_serial_port(void) { return 1; }
static unsigned burst_serial_baud(void) { return 2000000; }
#define spectrum_acquire NULL
static bool spectrum_command(const char *s,unsigned f,void *acquire){return false;}
static void reply(const char *s) { snprintf(response,sizeof(response),"%s",s); }
static bool gain_command(const char *s) { return false; }
static unsigned gain_max(void) { return 79; }
#include "burst_limits.h"
static bool capture(unsigned n,unsigned d,unsigned f) {
    if(d<6 || d>9){reply("ERR rate\n");return false;}
    ++captures;last_samples=n;last_format=f;return true;
}
static void vTaskDelay(int ticks) {}
static void prepare_rx(void) { rx_ready=true; }
'''
        checks = r'''
static void command(const char *s) { char line[128];snprintf(line,sizeof(line),"%s",s);handle_command(line); }
int main(void) {
 command("INFO");assert(!strcmp(response,"H2SDR 6 burst 16380\n"));
 command("CAPS");assert(strstr(response,"RXLIMITS") && strstr(response,"SERIALLEASE") && strstr(response,"HWAGC"));
 assert(strstr(response,"TUNEEXT") && strstr(response,"LPFANA") && !strstr(response,"RX40"));
 command("LIMITS?");assert(strstr(response,"\"bandwidth\":[4,11,1,0]") && strstr(response,"\"rates\":[32000000,16000000,10666667,6400000]"));
 command("TRANSPORT?");assert(!strcmp(response,"TRANSPORT UART 2000000\n"));
 command("SYNC 987654321");assert(!strcmp(response,"SYNC 987654321\n"));
 command("CAP16 16380 7");assert(captures==1 && last_samples==16380 && last_format==16);
 command("CAP20 257 7");assert(captures==2 && last_samples==257 && last_format==20);
 command("RXRUN 256 7 2 20");assert(captures==4 && !strcmp(response,"END\n"));
 for(unsigned d=0;d<=5;d++){
   char c[40];snprintf(c,sizeof(c),"CAP20 256 %u",d);command(c);assert(!strcmp(response,"ERR rate\n"));
 }
 const char *bad[]={"CAP16 16381 7","CAP20 255 7","CAP20 256 7 junk",
 "RXRUN 256 7 0 20","RXRUN 256 7 1001 20","RXRUN 256 7 2 32","FREQ 2412 junk",
 "FREQ 2412.5","FREQ 99","FREQ 6001","FREQ 6001","GAIN AUTO","BANDWIDTH 3","BANDWIDTH 12","LPF 128","BANDWIDTH 20 junk",
 "TX20 256 1000000 0","CW START","REPLAY20 256 40000000 0"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){command(bad[i]);assert(!strcmp(response,"ERR command\n"));}
 assert(captures==4);
 for(unsigned d=6;d<=9;d++){
   char c[40];snprintf(c,sizeof(c),"CAP20 256 %u",d);command(c);
 }
 assert(captures==8);
 command("BANDWIDTH 4");assert(rx_filter==112 && !strcmp(response,"OK\n"));
 command("BANDWIDTH 11");assert(rx_filter==0);
 command("BANDWIDTH 0");assert(rx_filter==0);
 command("LPF 127");assert(rx_filter==127);
 command("LPF?");assert(!strcmp(response,"LPF 127 45\n"));
 command("LPF AUTO");assert(rx_filter==-1);
 for(unsigned f=2412;f<=2472;f+=5){
   char c[40];snprintf(c,sizeof(c),"FREQ %u",f);command(c);assert(frequency_mhz==f && rx_ready);
 }
 command("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 for(unsigned f=100;f<=6000;f++){
   char c[40];snprintf(c,sizeof(c),"FREQ %u",f);command(c);assert(frequency_mhz==f);
   bool channel=(f>=2412 && f<=2472 && (f-2412)%5==0)||f==2484;
   unsigned before=pll_writes;tune_rx(f);
   assert(calibrated_mhz==(channel?f:2412));
   assert(pll_writes==before+!channel);if(!channel)assert(pll_mhz==f);
 }
 command("FREQ 2484");assert(frequency_mhz==2484 && !strcmp(response,"OK\n"));
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'commands.c'
            path.write_text(stub + frequency + handler + checks)
            binary = Path(tmp) / 'commands'
            subprocess.run(['cc', '-std=c11', '-I'+str(root/'main/common'), str(path), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
