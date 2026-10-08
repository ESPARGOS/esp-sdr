"""The ESP32 acquisition entry point must repair stale calibration before DMA."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class ESP32CaptureRecovery(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_capture_and_spectrum_recover_before_start(self):
        source = (Path(__file__).resolve().parents[1] / 'main/targets/esp32/receiver.c').read_text()
        code = source[source.index('static bool acquire_iq('):source.index('#include "ring_probe.h"')]
        stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define BIT(n) (1u << (n))
#define CAPACITY 512
#define MAX_SAMPLES CAPACITY
#define SENTINEL 0xdeadbeefu
#define DUMP_CTRL 0
#define DUMP_BYTES 1
#define DUMP_STATUS 2
#define DPORT_IRAM_DRAM_AHB_SEL_REG 3
#define DPORT_MAC_DUMP_MODE_M 3
#define DPORT_MAC_DUMP_MODE_S 0
static uint32_t samples[CAPACITY], regs[4];
static bool stale;
static unsigned preparations, starts;
static bool rx_recalibration_stale(void) { return stale; }
static void prepare_rx(void) { assert(stale); stale=false; preparations++; }
static void reply(const char *format, ...) { }
static void filter_apply(void) { }
static void filter_restore(void) { }
static void esp_rom_delay_us(unsigned n) { }
static int64_t esp_timer_get_time(void) { static int64_t t; return ++t; }
static void write_reg(unsigned reg, uint32_t value) {
    assert(!stale);
    regs[reg]=value;
    if(reg==DUMP_CTRL && (value & BIT(19))) {
        starts++;
        unsigned n=value & 0xffff;
        for(unsigned i=0;i<n;i++) samples[i]=i;
        regs[DUMP_CTRL]|=BIT(18);
        regs[DUMP_STATUS]=n;
    }
}
#define REG_READ(r) regs[r]
#define REG_WRITE(r,v) write_reg(r,v)
#define REG_SET_BIT(r,b) write_reg(r,regs[r]|(b))
#define REG_CLR_BIT(r,b) write_reg(r,regs[r]&~(b))
#define DPORT_REG_READ(r) REG_READ(r)
#define DPORT_REG_WRITE(r,v) REG_WRITE(r,v)
'''
        checks = r'''
int main(void) {
 unsigned elapsed; const uint32_t *data;
 stale=true;
 assert(!acquire_iq(255,0,1,&elapsed));
 assert(stale && preparations==0 && starts==0);
 assert(acquire_iq(256,0,1,&elapsed));
 assert(!stale && preparations==1 && starts==1);
 assert(acquire_iq(256,0,1,&elapsed));
 assert(preparations==1 && starts==2);
 stale=true;
 assert(spectrum_acquire(256,0,&data,&elapsed));
 assert(!stale && preparations==2 && starts==3 && data==samples);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / 'check.c').write_text(stub + code + checks)
            subprocess.run(['cc', '-std=c11', str(p / 'check.c'), '-o', str(p / 'check')], check=True)
            subprocess.run([str(p / 'check')], check=True)
