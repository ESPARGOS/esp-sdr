/* S3: keep a forced RX gain when the receiver switches to its second table.
 * With a strong signal a forced gain can silently land on an entry of the
 * high gain table (83..165). The PHY writes both tables through g_phyFuns
 * slot 11 (rom_write_gain_mem), so capture the calibrated words there and pin
 * the forced entry into every high slot; GAIN HARDWARE restores them. */
#include <stdint.h>

#define LOW 83u
typedef void (*write_fn)(uint32_t, uint32_t, uint32_t);
extern void **g_phyFuns;
extern void __real_set_rx_gain_table(uint32_t, uint32_t, uint32_t, uint32_t);
static write_fn rom_write;
static uint32_t words[2 * LOW][2];
static int pinned = -1;

static void capture(uint32_t w0, uint32_t w1, uint32_t i) {
    if (i < 2 * LOW) words[i][0] = w0, words[i][1] = w1;
    if (pinned >= 0 && i >= LOW && i < 2 * LOW) w0 = words[pinned][0], w1 = words[pinned][1];
    rom_write(w0, w1, i);
    if (pinned >= 0 && i == (uint32_t)pinned) /* recalibrated: re-pin */
        for (unsigned h = LOW; h < 2 * LOW; h++) rom_write(w0, w1, h);
}

void __wrap_set_rx_gain_table(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    if (g_phyFuns && g_phyFuns[11] != (void *)capture) {
        rom_write = (write_fn)g_phyFuns[11];
        g_phyFuns[11] = (void *)capture;
    }
    __real_set_rx_gain_table(a, b, c, d);
}

void burst_gain_pin_s3(int index) {
    if (!rom_write) return;
    pinned = -1;
    for (unsigned h = LOW; h < 2 * LOW; h++) {
        unsigned src = index >= 0 ? (unsigned)index : h;
        rom_write(words[src][0], words[src][1], h);
    }
    pinned = index;
}
