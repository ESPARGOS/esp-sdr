"""Exercise production snapshot accumulation/encoding with deterministic FFT output."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SnapshotSpectrum(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_mean_max_bin_order_and_frame_counts(self):
        source = (ROOT / 'main/common/spectrum.c').read_text()
        # Model the C5's reserved lower bank independently of host addresses.
        for address, offset in [('0x40820000', 0), ('0x40822000', 8192),
                                ('0x40823000', 12288), ('0x40825000', 20480)]:
            for ctype in ['int16_t', 'float', 'uint8_t']:
                source = source.replace(f'({ctype} *){address}',
                                        f'({ctype} *)(host_c5_sram + {offset})')
        sdk_headers = ['esp_cpu.h', 'dsps_fft2r.h', 'esp_rom_crc.h', 'esp_timer.h',
                       'freertos/FreeRTOS.h', 'freertos/task.h', 'spectrum_fft.h']
        for header in sdk_headers:
            source = source.replace(f'#include "{header}"', '')
        stub = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "spectrum_stats.h"
#include "burst_serial.h"
#define ESP_OK 0
unsigned spectrum_dc_mode;
static unsigned transforms,frames,wanted_units,wanted_n,wanted_detector,full_scale,stop_after, release_at;
static unsigned batch_start;
static char replies[4096];
static uint32_t words[2048];
static uint8_t host_c5_sram[65536] __attribute__((aligned(16)));
static int16_t *dsps_fft_w_table_sc16;
static uint32_t esp_cpu_get_cycle_count(void){return 0;}
static int64_t esp_timer_get_time(void){static int64_t t;return t+=1000;}
static void vTaskDelay(unsigned ticks){}
static unsigned dsps_fft2r_init_sc16(int16_t *table,unsigned n){return ESP_OK;}
static void dsps_fft2r_sc16_ansi(int16_t *x,unsigned n){
    for(unsigned j=0;j<n;j++){x[2*j]=full_scale?INT16_MIN:(j%97+1)*(transforms+1);x[2*j+1]=full_scale?INT16_MIN:0;}
    transforms++;
}
static void spectrum_fft_poll(int16_t *x,unsigned n,const int16_t *w,void (*poll)(void*),void *context){
 dsps_fft2r_sc16_ansi(x,n);poll(context);
}
static uint32_t esp_rom_crc32_le(uint32_t c,const void *p,unsigned n){return 0;}
static burst_serial_port_t test_port=BURST_SERIAL_USB;
burst_serial_port_t burst_serial_port(void){return test_port;}
bool burst_serial_stop_requested(void){return stop_after && transforms>=stop_after;}
void spectrum_stats_init(spectrum_stats_t *s){memset(s,0,sizeof(*s));}
void spectrum_stats_emit(spectrum_stats_t *s,unsigned n,unsigned fs,uint32_t ffts,
 unsigned abandoned,uint32_t drops,uint32_t late,unsigned queue,bool (*send)(const void*,size_t)){}
'''
        check = r'''
static unsigned test_reverse(unsigned x,unsigned bits){unsigned r=0;while(bits--){r=r*2+(x&1);x>>=1;}return r;}
static bool acquire(unsigned n,unsigned rate,const uint32_t **p,unsigned *us){*p=words;*us=1;return true;}
bool burst_serial_send(const void *data,size_t size){
 const uint8_t *b=data;
 if(size<4||memcmp(b,"SPC1",4)){
  assert(strlen(replies)+size<sizeof(replies));
  strncat(replies,data,size);return true;
 }
 frames++;assert(size==wanted_n+32);
 wanted_units=b[20]|b[21]<<8;assert(wanted_units>0);
 assert((b[20]|b[21]<<8)==wanted_units);
 assert((b[22]&1)==wanted_detector);assert(b[27]==2);
 unsigned pairs=b[16]|b[17]<<8|b[18]<<16|b[19]<<24;
 assert(pairs==wanted_n*wanted_units);
 for(unsigned j=1;j<wanted_n;j++){
  if(j==wanted_n/2||j==wanted_n-1)continue; /* DC correction neighbors */
  double p=0;
  for(unsigned u=1;u<=wanted_units;u++){
   double v=(j%97+1)*(u+batch_start);v=full_scale?2147483648.0:v*v;
   p=wanted_detector?fmax(p,v):p+v;
  }
  if(!wanted_detector)p/=wanted_units;
  int expected=(int)lrint(20*log10(p));if(expected>255)expected=255;
  assert(abs((int)b[28+test_reverse(j,b[26])]-expected)<=1);
 }
 batch_start+=wanted_units;return true;
}
size_t burst_serial_try_send(const void *data,size_t size){
 if(transforms<release_at)return 0;
 assert(burst_serial_send(data,size));return size;
}
int main(void){
 assert(spectrum_command("SPECINFO?",2412,acquire));
 assert(strstr(replies,"[80000000,0,2048,1,1,0]"));
#if CONFIG_IDF_TARGET_ESP32C61
 assert(strstr(replies,"[4000000,5,2048,1,1,0]"));
 assert(strstr(replies,"[80000000,0,256,1,1,1]"));
#elif CONFIG_IDF_TARGET_ESP32C5
 assert(strstr(replies,"[4000000,5,2048,1,1,0]"));
 assert(strstr(replies,"[4000000,5,256,1,1,1]"));
 assert(strstr(replies,"[8000000,4,256,1,1,1]"));
 assert(strstr(replies,"[80000000,0,256,1,1,0]"));
#else
 assert(strstr(replies,"[80000000,0,256,1,1,0]"));
#endif
#if CONFIG_IDF_TARGET_ESP32C5
 test_port=BURST_SERIAL_UART;replies[0]=0;
 assert(spectrum_command("SPECINFO?",2412,acquire));
 assert(strstr(replies,"[4000000,5,256,1,1,0]"));
 assert(strstr(replies,"[8000000,4,256,1,1,0]"));
 test_port=BURST_SERIAL_USB;
#endif
 replies[0]=0;
 assert(spectrum_command("SPEC 1 1 1 0 0 4096",2412,acquire));
 assert(!strcmp(replies,"ERR spec_args\n"));
 for(full_scale=0;full_scale<=1;full_scale++)
 for(wanted_n=256;wanted_n<=2048;wanted_n*=2)
 for(unsigned legacy_units=1;legacy_units<=8;legacy_units++)
 for(wanted_detector=0;wanted_detector<=1;wanted_detector++)
 for(release_at=0;release_at<=5;release_at+=5){
  batch_start=transforms=frames=0;stop_after=6;replies[0]=0;char command[64];
  snprintf(command,sizeof(command),"SPEC 0 1 %u %u 0 %u",legacy_units,wanted_detector,wanted_n);
  assert(spectrum_command(command,2412,acquire));
  assert(transforms==6 && batch_start==6);
  assert(frames==(release_at?3:6));
 }
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'snapshot.c'
            src.write_text('#include <stdlib.h>\n' + stub + source + check)
            exe = Path(tmp) / 'snapshot'
            for target in ['ESP32C3', 'ESP32C61', 'ESP32C5']:
                with self.subTest(target=target):
                    subprocess.run(['cc', '-std=gnu11', '-O2', '-fsanitize=undefined',
                                    f'-DCONFIG_IDF_TARGET_{target}=1',
                                    '-I' + str(ROOT / 'main/common'), str(src), '-lm', '-o', str(exe)], check=True)
                    subprocess.run([str(exe)], check=True)
