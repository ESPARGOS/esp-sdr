"""Run the shared production parser for both C5 and C61 with hardware stubs."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

class SharedCommands(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_both_chips(self):
        source=(Path(__file__).resolve().parents[1]/'main/families/c5_c6_c61/receiver.c').read_text()
        handler=source[source.index('static void handle_command(char *line) {'):]
        stub=r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
static bool burst_version_command(const char *s) { return false; }
static bool burst_gpio_command(const char *s) { return false; }
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#define BURST_SERIAL_UART 1
#define IQ_WORDS 16380u
#define RX_FILTER_REG 4u
#include "rx_bandwidth.h"
#include "rx_tuning.h"
#define CONFIG_ESP_SDR_UART_ENABLED 1
#if CONFIG_IDF_TARGET_ESP32C61
#define BURST_ID "C61SDR"
#else
#define BURST_ID "C5SDR"
#endif
static unsigned frequency_mhz,captures,last_format,last_samples;
static int rx_filter,rx_analog_filter;
static bool rx_ready;
static unsigned rx_channel_mode,preparations;
static char response[256];
static int burst_serial_port(void) { return 1; }
static unsigned burst_serial_baud(void) { return 921600; }
#define spectrum_acquire NULL
static bool spectrum_command(const char *s,unsigned f,void *acquire){return false;}
static bool ring_test(const char *s){return false;}
static void reply(const char *s) { snprintf(response,sizeof(response),"%s",s); }
static bool gain_command(const char *s) { return false; }
static unsigned gain_max(void) { return 84; }
#include "burst_limits.h"
static bool capture(unsigned n,unsigned d,unsigned f) { ++captures;last_samples=n;last_format=f;return true; }
static void vTaskDelay(int ticks) {}
static unsigned phy_chip_i2c_readReg(unsigned a,unsigned b,unsigned c) { return 4; }
#define REG_READ(a) 0u
#if CONFIG_IDF_TARGET_ESP32C61
static unsigned calibrated,pll,tunes;
unsigned phy_i2c_readReg(unsigned b,unsigned h,unsigned r){assert(b==0x62 && h==1 && r==12);return 0;}
void phy_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){assert(0);}
void esp_rom_delay_us(unsigned us){(void)us;}
void phy_chip_set_chan(unsigned f,unsigned mode) { assert(mode==0);calibrated=f;pll=f;tunes++; }
void phy_set_freq(unsigned f,int offset) { assert(offset==0);pll=f; }
#include "tuning.h"
#else
static bool frequency_valid(unsigned f) {return rx_frequency_valid(f);}
#endif
static void prepare_rx(void) {
 preparations++;
#if CONFIG_IDF_TARGET_ESP32C61
 phy_chip_set_chan(frequency_mhz,0);
#endif
 rx_ready=true;
}
'''
        check=r'''
static void command(const char *s) { char line[128];snprintf(line,sizeof(line),"%s",s);handle_command(line); }
int main(void) {
 command("INFO");assert(!strcmp(response,BURST_ID " 6 burst 16380\n"));
 command("CAPS");assert(strstr(response,"SERIALLEASE"));assert(strstr(response,"IQ8"));assert(strstr(response,"TUNEEXT"));
 command("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 for(unsigned f=100;f<=6000;f++){char cmd[32];snprintf(cmd,sizeof(cmd),"FREQ %u",f);command(cmd);assert(!strcmp(response,"OK\n") && frequency_mhz==f);}
 command("CAPS");assert(strstr(response,"DUALSERIAL"));
 command("TRANSPORT?");assert(!strcmp(response,"TRANSPORT UART 921600\n"));
 command("LIMITS?");
 assert(strstr(response,CONFIG_IDF_TARGET_ESP32C61?"[13,54,1,0]":"[11,48,1,0]"));
 assert(strstr(response,"80000000,40000000,20000000,10000000,8000000,4000000"));
 command("SYNC 987654321");assert(!strcmp(response,"SYNC 987654321\n"));
 command("CAP16 16380 5");assert(captures==1 && last_samples==16380 && last_format==16);
 command("CAP20 257 0");assert(captures==2 && last_samples==257 && last_format==20);
 command("RXRUN 256 3 2 20");assert(captures==4 && !strcmp(response,"END\n"));
 const char *bad[]={"CAP16 16381 0","CAP20 255 0","CAP20 16380 6","CAP20 256 0 junk",
 "RXRUN 256 0 0 20","RXRUN 256 0 1001 20","RXRUN 256 0 2 32","FREQ 5180 junk",
 "FREQ 2399.5","GAIN AUTO","TX20 256 1000000 0","CW START","REPLAY20 256 40000000 0"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){command(bad[i]);assert(!strcmp(response,"ERR command\n"));}
 assert(captures==4);
 command("FREQ 2412");assert(frequency_mhz==2412 && rx_ready);
 command("FREQ 5180");assert(frequency_mhz==5180);
 command("LPF 12");assert(rx_filter==12);command("LPF AUTO");assert(rx_filter==-1);
 command("ALPF 63");assert(rx_analog_filter==63);command("ALPF AUTO");assert(rx_analog_filter==-1);
#if CONFIG_IDF_TARGET_ESP32C61
 command("CAPS");assert(strstr(response,"TUNEEXT"));
 command("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 for(unsigned f=100;f<=6000;f++) {
   char cmd[32];snprintf(cmd,sizeof(cmd),"FREQ %u",f);command(cmd);
   assert(!strcmp(response,"OK\n") && frequency_mhz==f && pll==f && rx_ready);
   assert(calibrated==(((f>=2412 && f<=2472 && (f-2412)%5==0)||f==2484)?f:2412));
 }
 unsigned before=tunes;
 const char *invalid[]={"FREQ 99","FREQ 6001","FREQ -1","FREQ 0","FREQ 2413 junk"};
 for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++){command(invalid[i]);assert(!strcmp(response,"ERR command\n"));}
 assert(tunes==before);
 command("FREQ 2413");command("FREQ 2412");assert(pll==2412 && calibrated==2412);
 command("BANDWIDTH 21");assert(rx_analog_filter==24);
 command("BANDWIDTH 13");assert(rx_analog_filter==60);
 command("BANDWIDTH 54");assert(rx_analog_filter==0);
 command("BANDWIDTH 0");assert(rx_analog_filter==0);
 command("BANDWIDTH 12");assert(!strcmp(response,"ERR command\n"));
 command("BANDWIDTH 55");assert(!strcmp(response,"ERR command\n"));
#else
 command("BANDWIDTH 20");assert(!strcmp(response,"OK\n") && rx_analog_filter==12);
 command("BANDWIDTH 11");assert(rx_analog_filter==60);
 command("BANDWIDTH 23");assert(rx_analog_filter==0);
 command("BANDWIDTH 0");assert(rx_analog_filter==0);
 command("BANDWIDTH 10");assert(!strcmp(response,"ERR command\n"));
 unsigned before=preparations;
 command("BANDWIDTH 24");assert(!strcmp(response,"OK\n") && rx_channel_mode==1 && rx_analog_filter==52 && preparations==before);
 command("BANDWIDTH 40");assert(rx_channel_mode==1 && rx_analog_filter==12);
 command("BANDWIDTH 48");assert(rx_channel_mode==1 && rx_analog_filter==0);
 command("FREQ 5500");assert(rx_channel_mode==1 && rx_ready);
 before=preparations;
 command("BANDWIDTH 23");assert(rx_channel_mode==0 && rx_analog_filter==0 && preparations==before+1);
 command("BANDWIDTH 11");assert(rx_channel_mode==0 && rx_analog_filter==60 && preparations==before+1);
 command("BANDWIDTH 0");assert(rx_channel_mode==1 && rx_analog_filter==0 && preparations==before+2);
 command("BANDWIDTH 49");assert(!strcmp(response,"ERR command\n") && rx_channel_mode==1 && rx_analog_filter==0);
 command("BANDWIDTH 23 junk");assert(!strcmp(response,"ERR command\n") && rx_channel_mode==1);
#endif
 command("BANDWIDTH 21 junk");assert(!strcmp(response,"ERR command\n"));
 command("RELEASE");assert(!strcmp(response,"OK\n"));
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'commands.c';path.write_text(stub+handler+check)
            for c61 in [0,1]:
                binary=Path(tmp)/f'commands-{c61}'
                subprocess.run(['cc','-std=c11','-DCONFIG_IDF_TARGET_ESP32C6=0','-I'+str(Path(__file__).resolve().parents[1]/'main/targets/esp32c61'),'-I'+str(Path(__file__).resolve().parents[1]/'main/common'),f'-DCONFIG_IDF_TARGET_ESP32C61={c61}',f'-DCONFIG_IDF_TARGET_ESP32C5={1-c61}',str(path),'-o',str(binary)],check=True)
                subprocess.run([str(binary)],check=True)
