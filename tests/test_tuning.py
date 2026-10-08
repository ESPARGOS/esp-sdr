"""Host checks of production tuning helpers/parsers; no RF hardware is used."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

MAIN = Path(__file__).resolve().parents[1] / 'main'

@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class Tuning(unittest.TestCase):
    def compile_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'check.c'
            exe = Path(tmp) / 'check'
            c.write_text(source)
            subprocess.run(['cc', '-std=c11', '-Werror=implicit-function-declaration',
                            '-I'+str(MAIN/'common'), '-I'+str(MAIN), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_c5_exact_frequency_and_bandwidth_mode(self):
        self.compile_run(r'''
#include <assert.h>
static unsigned tuned, selected_mode;
void phy_set_chanfreq(unsigned f, unsigned mode) { tuned=f; selected_mode=mode; }
#include "targets/esp32c5/tuning.h"
int main(void) {
    for (unsigned mode=0; mode<=1; mode++)
        for (unsigned mhz=100; mhz<=6000; mhz++) {
            c5_set_chan(mhz,mode);
            assert(tuned==mhz && selected_mode==mode);
        }
}
''')

    def test_receive_frequency_maps_to_each_chip_pll(self):
        stub = r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
#include <stdint.h>
#include "rx_tuning.h"
static unsigned calibrated, pll, writes;
static int pll_offset;
static unsigned ckgen=0x63, cap=0xb0;
unsigned ram_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){assert(r==0);assert((b==0x65 && h==4)||(b==0x62 && h==1));return b==0x65?ckgen:cap;}
void ram_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){assert(r==0);assert((b==0x65 && h==4)||(b==0x62 && h==1));if(b==0x65)ckgen=v;else cap=v;}
unsigned rom_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){assert(h==1 && ((b==0x65 && r==0)||(b==0x62 && r==16)));return ckgen;}
void rom_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){assert(h==1 && ((b==0x65 && r==0)||(b==0x62 && r==16)));ckgen=v;}
#define rom1_chip_i2c_readReg rom_chip_i2c_readReg
#define rom1_chip_i2c_writeReg rom_chip_i2c_writeReg
unsigned phy_i2c_readReg(unsigned b,unsigned h,unsigned r){assert(b==0x62 && h==1 && r==12);return 0;}
void phy_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){assert(0);}
void esp_rom_delay_us(unsigned us){(void)us;}
unsigned char phy_param[50] = {[49]=2};
void phy_chip_set_chan(unsigned f,unsigned m){assert(m==0);calibrated=pll=f;}
void chip_v7_set_chan(unsigned f,unsigned m){phy_chip_set_chan(f,m);}
void set_chanfreq(unsigned f,unsigned m){phy_chip_set_chan(f,m);}
void phy_set_chanfreq(unsigned f,unsigned m){phy_chip_set_chan(f,m);}
void phy_set_freq(unsigned f,int o){pll=f;pll_offset=o;writes++;}
void set_rf_freq_offset(unsigned c,unsigned f,int o){assert(c==0);phy_set_freq(f,o);}
void rom_set_rf_freq_offset(unsigned c,unsigned f,int o){assert(c==0);phy_set_freq(f,o);}
void phy_set_rf_freq_offset(unsigned c,unsigned f,int o){assert(c==phy_param[49]);phy_set_freq(f,o);}
unsigned rtc_clk_xtal_freq_get(void){return 40;}
'''
        helpers = {}
        for target, start, end, call in [
            ('esp32', 'static void tune_rx(', 'static bool capture_rate(', 'tune_rx(f)'),
            ('esp32c2', 'static void tune_rx(', '#define send_bytes', 'tune_rx(f)'),
            ('esp32h2', 'static void tune_rx(', '#define send_bytes', 'tune_rx(f)'),
            ('esp32c3', 'static void tune_rx(', '#define send_bytes', 'tune_rx(f)'),
            ('esp32s2', 'static void s2_tune(', '#define S2_FREQ_MIN', 's2_tune(f)'),
            ('esp32s3', 'static void s3_tune(', '#define S3_FREQ_MIN', 's3_tune(f)'),
        ]:
            source = (MAIN/'targets'/target/'receiver.c').read_text()
            helpers[target] = (source[source.index(start, source.index(start)+1) if target == "esp32" else source.index(start):source.index(end)], call)
        source = (MAIN/'targets/esp32c6/chip.h').read_text()
        helpers['esp32c6'] = (source[source.index('static void c6_set_chan('):source.index('#define phy_chip_set_chan')], 'c6_set_chan(f,0)')
        for target, call in [('esp32c5','c5_set_chan(f,0)'), ('esp32c61','c61_set_chan(f,0)'), ('esp32s31','s31_tune(f)')]:
            helpers[target] = (f'#include "targets/{target}/tuning.h"\n', call)
        for target, (helper, call) in helpers.items():
            with self.subTest(target=target):
                lo = target in ('esp32','esp32s2','esp32s3','esp32c2','esp32c3','esp32c6')
                setup = f'#define CONFIG_IDF_TARGET_{target.upper()} 1\n'
                setup += '#include "rx_lo.h"\n' if lo else ''
                if target=='esp32s2':setup += 'static bool s2_pll_calibrate(void){return true;}\n'
                self.compile_run(stub+setup+helper+r'''
int main(void){
 assert(!rx_frequency_valid(99) && !rx_frequency_valid(6001));
 for(unsigned f=100;f<=6000;f++){
  assert(rx_frequency_valid(f));
  CALL;
  unsigned expected_khz=f*1000u;
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6
  if(f>=1842 && f<2210)expected_khz=f*1200u;
  assert(ckgen==0x63); /* Calibration must leave the selector in normal mode. */
#endif
#if CONFIG_IDF_TARGET_ESP32
  int64_t error=((int64_t)pll*1024+pll_offset)*1000-(int64_t)expected_khz*1024;
  assert(error>=-500 && error<=500);
  if(pll!=2412)assert(!(cap&128));
#else
  assert(pll*1000u+pll_offset==expected_khz);
#endif
  bool channel=(f>=2412 && f<=2472 && (f-2412)%5==0)||f==2484;
  assert(calibrated==EXPECTED);
 }
#if CONFIG_IDF_TARGET_ESP32S3
 for(int offset=-100;offset<=100;offset+=100){
  s3_fofs=offset;
  pll_offset=0;
  s3_tune(2412);
  assert(pll==2412 && pll_offset==offset);
  s3_tune(2001);
  assert(pll==2401 && pll_offset==200+offset);
  assert(ckgen==0x63);
 }
#endif
#if !CONFIG_IDF_TARGET_ESP32C5
 assert(writes>5800);
#endif
}
'''.replace('CALL',call).replace('EXPECTED','f' if target=='esp32c5' else '(channel?f:2412u)'))

    def test_alternate_lo_setup_and_return_to_normal(self):
        for target, tune in [('esp32s2', 's2_tune'), ('esp32s3', 's3_tune'), ('esp32c3', 'tune_rx'), ('esp32c2', 'tune_rx'), ('esp32c6', 'c6_tune')]:
            path = MAIN/'families/c5_c6_c61/receiver.c' if target=='esp32c6' else MAIN/'targets'/target/'receiver.c'
            source = path.read_text()
            start = source.index('static void prepare_rx(void) {')
            prepare = source[start:source.index('\n}', start)+2]
            with self.subTest(target=target):
                self.compile_run(f'#define CONFIG_IDF_TARGET_{target.upper()} 1\n'+r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
static unsigned ckgen, frequency_mhz, setups, delays;
static bool rx_ready;
enum {rx_prep=3, WIFI_SECOND_CHAN_NONE=0};
unsigned rom_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){assert(h==1 && (NEW_SELECTOR ? b==0x62 && r==16 : b==0x65 && r==0));return ckgen;}
void rom_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){assert(h==1 && (NEW_SELECTOR ? b==0x62 && r==16 : b==0x65 && r==0));ckgen=v;}
#define rom1_chip_i2c_readReg rom_chip_i2c_readReg
#define rom1_chip_i2c_writeReg rom_chip_i2c_writeReg
#include "rx_lo.h"
static void TUNE(unsigned f){assert(f==frequency_mhz);rx_lo_select(false);setups++;}
static void check_normal(void){assert(!(ckgen&RX_LO_MASK));}
#if CONFIG_IDF_TARGET_ESP32C6
static void phy_chip_set_chan(unsigned f,unsigned mode){assert(mode==0);TUNE(f);}
#endif
static void on(unsigned v){assert(v==1);check_normal();}
#define phy_stop_tx_tone on
#define phy_rom_stop_tx_tone on
#define stop_tx_tone on
#define phy_pbus_workmode check_normal
#define rom_pbus_workmode check_normal
#define phy_pbus_xpd_tx_off check_normal
#define rom_pbus_xpd_tx_off check_normal
#define phy_pbus_xpd_rx_on on
#define rom_pbus_xpd_rx_on on
#define phy_set_rxclk_en on
#define rom_set_rxclk_en on
#define set_rxclk_en on
#define gain_apply check_normal
static void esp_rom_delay_us(unsigned n){assert(n==3000);delays++;}
static void esp_wifi_set_channel(unsigned a,unsigned b){assert(0);}
static void force_rx_gain(unsigned a,unsigned b,unsigned c){assert(0);}
'''.replace('TUNE',tune).replace('NEW_SELECTOR', '1' if target in ('esp32c2','esp32c6') else '0')+prepare+r'''
int main(void){
 const unsigned frequencies[]={2412,2000,2001,1841,1842,2209,2210,2004,2484,6000};
 for(unsigned value=0;value<256;value++){
  ckgen=value;rx_lo_select(true);assert(ckgen==(value|RX_LO_MASK));
  rx_lo_select(false);assert(ckgen==(value&~RX_LO_MASK));
 }
 for(unsigned n=0;n<sizeof(frequencies)/sizeof(frequencies[0]);n++){
  frequency_mhz=frequencies[n];rx_ready=false;ckgen=0xe3;
  unsigned before=setups;prepare_rx();
  assert(rx_ready && setups==before+1);
  assert(ckgen==(rx_lo_plan(frequency_mhz).alternate?(0xe3|RX_LO_MASK):0xe3));
  before=delays;prepare_rx();assert(delays==before);
 }
}
''')

    def test_s2_capacitor_search_and_failure_restore(self):
        self.compile_run(r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
static unsigned regs[16], steps, scenario;
unsigned rom_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){
 assert(b==0x62 && h==1 && r<16);
 if(r==12){
  unsigned cap=regs[1]+((regs[2]&16)<<4);
  assert((regs[11]&64) && (regs[0]&128));
  bool locked=scenario==0?((cap>=10 && cap<=11)||(cap>=254 && cap<=262)):
              scenario==1?(cap>=509):false;
  return locked?0:4;
 }
 return regs[r];
}
void rom_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){
 assert(b==0x62 && h==1 && r<16 && v<256);regs[r]=v;
}
void esp_rom_delay_us(unsigned n){assert(n==20);steps++;}
#include "targets/esp32s2/pll.h"
int main(void){
 for(scenario=0;scenario<3;scenario++){
  regs[0]=0x30;regs[1]=173;regs[2]=0x88;regs[11]=0x28;steps=0;
  bool ok=s2_pll_calibrate();assert(ok==(scenario<2));
  assert(steps==(ok?513:512));
  if(ok){
   assert(regs[1]+((regs[2]&16)<<4)==(scenario==0?258:510));
   assert((regs[2]&~16u)==0x88 && regs[0]==0xb0 && regs[11]==0x68);
  } else assert(regs[0]==0x30 && regs[1]==173 && regs[2]==0x88 && regs[11]==0x28);
 }
}
''')

    def test_c5_bandwidth_mode_survives_prepare_and_retuning(self):
        source = (MAIN/'families/c5_c6_c61/receiver.c').read_text()
        prepare = source[source.index('static void prepare_rx(void) {'):source.index('#include "filter_probe.h"')]
        self.compile_run(r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
#define CONFIG_IDF_TARGET_ESP32C5 1
#define CONFIG_IDF_TARGET_ESP32C61 0
#define CONFIG_IDF_TARGET_ESP32C6 0
static unsigned frequency_mhz,rx_channel_mode,calls,calibrated,pll,last_mode;
static bool rx_ready;
static int rx_filter=-1;
unsigned char phy_param[50]={[49]=2};
void phy_set_chanfreq(unsigned f,unsigned m){calibrated=pll=f;last_mode=m;calls++;}
void phy_set_rf_freq_offset(unsigned c,unsigned f,int o){assert(c==2 && o==0);pll=f;}
void phy_stop_tx_tone(unsigned x){assert(x==1);}
void phy_pbus_workmode(void){}
void phy_pbus_xpd_tx_off(void){}
void phy_pbus_xpd_rx_on(unsigned x){assert(x==1);}
void phy_set_rxclk_en(unsigned x){assert(x==1);}
void phy_rx_filter_mode(unsigned x){assert(x==12);}
void gain_apply(void){}
#include "targets/esp32c5/tuning.h"
''' + prepare + r'''
int main(void){
 for(unsigned mode=0;mode<=1;mode++)for(unsigned f=2300;f<=5500;f+=3200){
  frequency_mhz=f;rx_channel_mode=mode;rx_ready=false;
  unsigned before=calls;prepare_rx();
  assert(calls==before+1 && last_mode==mode && pll==f && rx_ready);
  assert(calibrated==f);
  prepare_rx();assert(calls==before+1);
 }
 rx_filter=12;rx_ready=false;prepare_rx();assert(last_mode==1);
}
''')

    def test_s2_s3_s31_production_parsers(self):
        stub = r'''
#include <assert.h>
#include <stdbool.h>
bool rx_recalibration_stale(void) { return false; }
void rx_recalibrate(unsigned mhz) {}
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "rx_tuning.h"
#include "rx_bandwidth.h"
#define S2_FREQ_MIN RX_FREQ_MIN
#define S2_FREQ_MAX RX_FREQ_MAX
#define S3_FREQ_MIN RX_FREQ_MIN
#define S3_FREQ_MAX RX_FREQ_MAX
#define IQ_WORDS 16380u
#define BURST_SERIAL_UART 1
#define RX_GAIN 0
#define REG_READ(r) 0u
#define REG_WRITE(r,v) ((void)(v))
static unsigned gain_init, gain_threshold;
static void phy_set_txclk_en(unsigned on) {}
static void burst_gain_mirror(int gain) {}
static unsigned frequency_mhz,gain_max=72,gain_code;
static bool rx_ready,hardware_agc;
static int rx_filter;
static char response[256];
#define spectrum_acquire NULL
static bool s31_spectrum_command(const char *s){return false;}
static bool spectrum_command(const char *s,unsigned f,void *acquire){return false;}
static bool ring_test(const char *s){return false;}
static void reply(const char *fmt,...){va_list a;va_start(a,fmt);vsnprintf(response,sizeof(response),fmt,a);va_end(a);}
static int burst_serial_port(void){return 1;}
static unsigned burst_serial_baud(void){return 2000000;}
static bool gain_command(const char *s){return false;}
static bool ring_command(const char *s){return false;}
static bool burst_version_command(const char *s) { return false; }
static bool burst_gpio_command(const char *s){return false;}
static bool limits_command(const char *s){return false;}
static bool capture(unsigned n,unsigned d,unsigned f){return true;}
static bool capture_rate(unsigned n,unsigned d,unsigned f){return true;}
static void prepare_rx(void){rx_ready=true;}
static void apply_gain(void){}
static void s31_tune(unsigned f){assert(frequency_mhz==f);}
static void vTaskDelay(unsigned t){}
static unsigned rom_chip_i2c_readReg(unsigned a,unsigned b,unsigned c){return 0;}
static unsigned phy_i2c_readReg(unsigned a,unsigned b,unsigned c){return 0;}
'''
        for target in ['esp32s2','esp32s3','esp32s31']:
            with self.subTest(target=target):
                target_dir=MAIN/'targets'/target
                if target=='esp32s31':
                    target_dir /= 'burst'
                source=(target_dir/'receiver.c').read_text()
                name='command' if target=='esp32s31' else 'handle_command'
                handler=source[source.index('static void '+name+'('):source.index('void app_main(')]
                self.compile_run(f'#define CONFIG_IDF_TARGET_{target.upper()} 1\n'+stub+handler+r'''
static void send(const char *s){char line[128];snprintf(line,sizeof(line),"%s",s);HANDLER(line);}
int main(void){
 send("CAPS");assert(strstr(response,"TUNEEXT"));
#if CONFIG_IDF_TARGET_ESP32S31
 assert(strstr(response," SPEC ") && strstr(response," SPECN ") && strstr(response," SPECCAPS "));
#endif
 send("RANGE?");assert(!strcmp(response,"RANGE 100 6000 1\n"));
 for(unsigned f=100;f<=6000;f++){
  char cmd[40];snprintf(cmd,sizeof(cmd),"FREQ %u",f);send(cmd);
  assert(!strcmp(response,"OK\n") && frequency_mhz==f);
 }
 const char *invalid[]={"FREQ 99","FREQ 6001","FREQ 2612.5","FREQ -1","FREQ 2612 junk"};
 for(unsigned j=0;j<sizeof(invalid)/sizeof(invalid[0]);j++){send(invalid[j]);assert(!strcmp(response,"ERR command\n"));}
}
'''.replace('HANDLER',name))
