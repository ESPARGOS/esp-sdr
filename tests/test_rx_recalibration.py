"""Fresh measurements must override reference tunes only during calibration."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

MAIN = Path(__file__).resolve().parents[1] / 'main'


@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class Recalibration(unittest.TestCase):
    def test_h2_packs_each_rf_gain_without_overwriting_phy_parameters(self):
        vendor = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rx_recalibration.h"
_Alignas(4) unsigned char phy_param[2048];
uint32_t mock_iq_control=0x12345678;
static unsigned writes, measurements, forced=1;
static const uint16_t pairs[]={0x0102,0xfe03,0x04fc,0xfbfa};
void chip_v7_set_chan_ana(unsigned ch) {}
void phy_set_freq(unsigned mhz,int offset) {}
void force_rx_gain(unsigned force,unsigned gain) { forced=force; }
void set_rx_gain_cal_iq(void *out,unsigned debug) {
    assert(!forced);
    memcpy(out,pairs,sizeof(pairs));
    mock_iq_control|=1u<<29;
    measurements++;
}
extern void install_word(uint32_t,uint32_t,unsigned);
void write_gain_mem(uint32_t a,uint32_t b,unsigned index) {
    unsigned group=index%4;
    unsigned iq=((pairs[group]>>1)&0x1f80)|(pairs[group]&127);
    assert(a==0x12345 && (b&0x1fff)==iq);
    assert((b&~0x1fffu)==(0x01402000u|(group<<20)));
    writes++;
}
void set_rx_gain_table(void) {
    assert(!(*(uint32_t *)(phy_param+52)&0x100));
    assert(mock_iq_control==0x12345678);
    for(unsigned group=0;group<4;group++)
        install_word(0x12345,0x01402000u|(group<<20)|0x1fff,group);
}
int main(void) {
    memset(phy_param+60,0xa5,8);
    rx_h2_calibrate_iq_at_boot();
    rx_h2_calibrate_iq_at_boot(); /* Idempotent, even if startup is retried. */
    rx_recalibrate(2413);
    rx_recalibrate(2484);
    assert(measurements==1 && writes==8);
    for(unsigned i=60;i<68;i++) assert(phy_param[i]==0xa5);
}
'''
        reference = '''#include <stdint.h>
extern void write_gain_mem(uint32_t,uint32_t,unsigned);
void install_word(uint32_t a,uint32_t b,unsigned index) { write_gain_mem(a,b,index); }
'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/'sdkconfig.h').write_text('')
            (path/'soc').mkdir()
            (path/'soc/soc.h').write_text(
                '#include <stdint.h>\nextern uint32_t mock_iq_control;\n'
                '#define REG_READ(r) mock_iq_control\n'
                '#define REG_WRITE(r,v) (mock_iq_control=(v))\n')
            (path/'vendor.c').write_text(vendor)
            (path/'reference.c').write_text(reference)
            subprocess.run([
                'cc', '-Drx_recalibrate=rx_recalibrate_locked', '-std=c11', '-DCONFIG_IDF_TARGET_ESP32H2=1',
                '-I'+tmp, '-I'+str(MAIN/'common'), str(MAIN/'common/rx_recalibration.c'),
                str(path/'vendor.c'), str(path/'reference.c'),
                '-Wl,--wrap=chip_v7_set_chan_ana', '-Wl,--wrap=write_gain_mem',
                '-o', str(path/'check')
            ], check=True)
            subprocess.run([str(path/'check')], check=True)

    def test_legacy_preserve_iq_while_refreshing_dc(self):
        vendor = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rx_recalibration.h"
_Alignas(4) unsigned char phy_param[2048];
uint32_t chip7_sleep_params[128];
static unsigned wanted, tuned, tables, forced = 1;
static uint32_t *flags, dc_mask, iq_mask;
void mock_clear(unsigned reg, unsigned bits) {
#if CONFIG_IDF_TARGET_ESP32
    assert(reg == 0x3ff5c02cu);
#elif CONFIG_IDF_TARGET_ESP32C2
    assert(reg == 0x6004a02cu);
#elif CONFIG_IDF_TARGET_ESP32C6
    assert(reg == 0x600a702cu);
#else
    assert(reg == 0x6001c02cu);
#endif
    assert(bits == (1u << 23)); forced = 0;
}
void force_rx_gain(unsigned enable, unsigned index) { forced = enable; }
void chip_v7_set_chan_ana(unsigned channel) { tuned = channel; }
void phy_set_freq(unsigned mhz, int offset) { tuned = mhz; }
void set_rf_freq_offset(unsigned xtal, unsigned mhz, int offset) { tuned = mhz; }
void rom_set_rf_freq_offset(unsigned xtal, unsigned mhz, int offset) { tuned = mhz; }
unsigned rtc_clk_xtal_freq_get(void) { return 40; }
void esp_rom_delay_us(unsigned us) {}
#define I2C(prefix) \
 unsigned prefix##_chip_i2c_readReg(unsigned b,unsigned h,unsigned r){return 0;} \
 void prefix##_chip_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned v){}
I2C(ram)
I2C(rom)
I2C(rom1)
extern void reference_tune(unsigned);
static void rebuild(void) {
    assert(!forced);
    /* These are the vendor's gates for fresh DC and cached/factory IQ. */
    assert(!(*flags & dc_mask));
    assert((*flags & iq_mask) == iq_mask);
    reference_tune(6);
#if CONFIG_IDF_TARGET_ESP32H2
    assert(tuned == wanted);
#else
    assert(tuned == (wanted >= 1842 && wanted < 2210 ? wanted * 12 / 10 : wanted));
#endif
    *flags |= dc_mask;
    ++tables;
}
#if CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
void set_rx_gain_table(void) { rebuild(); }
#elif CONFIG_IDF_TARGET_ESP32C2
void set_rx_gain_table_new(unsigned mhz, unsigned a, unsigned b, unsigned c) { rebuild(); }
#else
void set_rx_gain_table(unsigned mhz, unsigned debug) { rebuild(); }
#endif
/* Any reintroduced direct IQ measurement must fail to link: its private
 * function is intentionally not provided by this vendor fixture. */
int main(void) {
    iq_mask = 0x400;
#if CONFIG_IDF_TARGET_ESP32
    flags = chip7_sleep_params; dc_mask = 0x20320;
#elif CONFIG_IDF_TARGET_ESP32S2
    flags = chip7_sleep_params; dc_mask = 0x20240;
#elif CONFIG_IDF_TARGET_ESP32C2
    flags = (void *)(phy_param + 328); dc_mask = 0x300;
#elif CONFIG_IDF_TARGET_ESP32C6
    flags = (void *)(phy_param + 164); dc_mask = 0x280;
#elif CONFIG_IDF_TARGET_ESP32H2
    flags = (void *)(phy_param + 52); dc_mask = 0x100; iq_mask = 0x200;
#else
    flags = (void *)(phy_param + 288); dc_mask = 0x200;
#endif
    /* H2's efuse IQ pair and adjacent PHY fields must not be overwritten by
     * the old four-pair loopback buffer. Also exercise repeated retunes. */
    memset(phy_param + 60, 0xa5, 8);
    const unsigned frequencies[] = {2413, 2150, 2484};
    for (unsigned i = 0; i < 3; ++i) {
        wanted = frequencies[i]; *flags = UINT32_MAX; forced = 1;
        rx_recalibrate(wanted);
        assert(tables == i + 1 && *flags == UINT32_MAX);
        reference_tune(6); assert(tuned == 6);
        for (unsigned j = 60; j < 68; ++j) assert(phy_param[j] == 0xa5);
    }
}
'''
        reference = '''extern void chip_v7_set_chan_ana(unsigned);
void reference_tune(unsigned channel) { chip_v7_set_chan_ana(channel); }
'''
        for chip in ('esp32', 'esp32c2', 'esp32c3', 'esp32c6',
                     'esp32s2', 'esp32s3'):
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                (path/'sdkconfig.h').write_text('')
                (path/'soc').mkdir()
                (path/'soc/soc.h').write_text(
                    'void mock_clear(unsigned,unsigned);\n'
                    '#define REG_CLR_BIT(r,b) mock_clear(r,b)\n')
                (path/'soc/rtc.h').write_text('unsigned rtc_clk_xtal_freq_get(void);\n')
                (path/'esp_rom_sys.h').write_text('void esp_rom_delay_us(unsigned);\n')
                (path/'vendor.c').write_text(vendor)
                (path/'reference.c').write_text(reference)
                subprocess.run([
                    'cc', '-Drx_recalibrate=rx_recalibrate_locked', '-std=c11', '-Wall', '-Werror=implicit-function-declaration',
                    '-DCONFIG_IDF_TARGET_' + chip.upper() + '=1',
                    '-I'+tmp, '-I'+str(MAIN/'common'), '-I'+str(MAIN/'targets'/chip),
                    str(MAIN/'common/rx_recalibration.c'), str(path/'vendor.c'),
                    str(path/'reference.c'), '-Wl,--wrap=chip_v7_set_chan_ana',
                    '-o', str(path/'check')
                ], check=True)
                subprocess.run([str(path/'check')], check=True)

    def test_measurement_frequency_and_cache_invalidation(self):
        vendor = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "rx_recalibration.h"
_Alignas(4) unsigned char phy_param[2048];
static unsigned wanted, tuned, dc, iq, tables;
static uint32_t *flags;
#if CONFIG_IDF_TARGET_ESP32C5
void phy_chip_set_chan_ana(unsigned mhz) { tuned=mhz; phy_param[42]=mhz>4000; }
/* Calls from a separate object exercise the same linker wrapping as libphy. */
extern void reference_tune(unsigned);
void phy_set_rx_gain_cal_dc(unsigned table,unsigned debug,void *rf,void *bb) {
    assert(!(*flags&0x280) && (*flags&0x400));
    reference_tune(2432); assert(tuned==wanted); dc++;
}
void phy_set_rx_gain_cal_iq(unsigned a,unsigned b,void *out,unsigned band,unsigned d,unsigned e) {
    assert(0); /* Retuning must not start loopback IQ transmission. */
}
void phy_adc_rate_cal_rxdc(void) {}
void phy_set_rx_gain_table(unsigned mhz,unsigned debug) {
    assert((*flags&0x680)==0x480 && mhz==wanted); tables++;
}
#else
void phy_set_channel_rfpll_freq(unsigned mhz,unsigned xtal,unsigned mode) {
    assert(xtal==40 && mode==0); tuned=mhz;
}
extern void reference_tune(unsigned);
unsigned phy_i2c_readReg(unsigned b,unsigned h,unsigned r) { assert(r==12); return 0; }
void phy_i2c_writeReg(unsigned b,unsigned h,unsigned r,unsigned value) { assert(0); }
void esp_rom_delay_us(unsigned us) {}
void phy_rxiq_cal_init(unsigned a,unsigned b,unsigned c) {
    assert(0); /* Retuning must preserve the installed IQ correction. */
}
void phy_set_rx_gain_table(unsigned mhz,unsigned debug) {
    assert(!(*flags&0x280) && (*flags&0x400));
    reference_tune(2484); assert(tuned==wanted); dc++; tables++;
}
#endif
int main(void) {
    flags=(void *)(phy_param+
#if CONFIG_IDF_TARGET_ESP32C5
        148
#else
        164
#endif
    );
    const unsigned frequencies[]={2413,5340,2413,2150};
    for(unsigned i=0;i<4;i++) {
        wanted=frequencies[i]; *flags=0xffffffff;
        unsigned old_dc=dc;
        rx_recalibrate(wanted);
        assert(dc>old_dc && iq==0 && tables==i+1);
        /* Unrelated calibration bits must survive; only RX caches expire. */
        assert((*flags&~0x680u)==(0xffffffffu&~0x680u));
        reference_tune(2437); assert(tuned==2437);
    }
}
'''
        reference = r'''
#if CONFIG_IDF_TARGET_ESP32C5
extern void phy_chip_set_chan_ana(unsigned);
void reference_tune(unsigned mhz) { phy_chip_set_chan_ana(mhz); }
#else
extern void phy_set_channel_rfpll_freq(unsigned,unsigned,unsigned);
void reference_tune(unsigned mhz) { phy_set_channel_rfpll_freq(mhz,40,0); }
#endif
'''
        for chip in ('ESP32C5', 'ESP32C61', 'ESP32S31'):
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                (path/'sdkconfig.h').write_text('')
                (path/'soc').mkdir()
                (path/'soc/soc.h').write_text('#define REG_CLR_BIT(r,b) ((void)(r), (void)(b))\n')
                (path/'esp_rom_sys.h').write_text('void esp_rom_delay_us(unsigned);\n')
                (path/'vendor.c').write_text(vendor)
                (path/'reference.c').write_text(reference)
                symbol = ('phy_chip_set_chan_ana' if chip == 'ESP32C5'
                          else 'phy_set_channel_rfpll_freq')
                subprocess.run([
                    'cc', '-Drx_recalibrate=rx_recalibrate_locked', '-std=c11', '-Wall', '-Werror=implicit-function-declaration',
                    '-DCONFIG_IDF_TARGET_'+chip+'=1', '-I'+tmp, '-I'+str(MAIN/'common'),
                    str(MAIN/'common/rx_recalibration.c'), str(path/'vendor.c'),
                    str(path/'reference.c'), '-Wl,--wrap='+symbol, '-o', str(path/'check')
                ], check=True)
                subprocess.run([str(path/'check')], check=True)
