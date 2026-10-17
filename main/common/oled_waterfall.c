/* ESP32-C3 SuperMini 0.42" OLED (SSD1306, 72x40 window of a 128x64 panel,
 * I2C 0x3C) live spectrum and waterfall.
 *
 * Each display line max-holds 256-point FFTs taken from 202 us, 80 MS/s
 * snapshots. FFTs are CPU-bound, so each snapshot is first scanned for
 * block energy and only its strongest blocks (plus one rotating block for
 * the floor) are transformed: short Wi-Fi/Bluetooth bursts are caught at
 * about ten times the duty cycle of transforming consecutive blocks. The central 216 bins (67.5 MHz) map to 72 columns,
 * three bins per column. Levels are relative to a tracked noise floor,
 * which also hides hardware AGC steps, and are ordered-dithered to 1 bit.
 *
 * BOOT button (GPIO9): short press steps the centre frequency, long press
 * cycles the layout. */
#include "oled_waterfall.h"
#include "burst_serial.h"
#include "spectrum_dc.h"
#include "spectrum_fft.h"
#include "spectrum_math.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "dsps_fft2r.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define W             72
#define H             40
#define PAGES         (H / 8)
#define FFT_N         256u
#define BINS_PER_COL  3u
#define FIRST_BIN     (FFT_N / 2 - W * BINS_PER_COL / 2) /* display-order bin */
#define SNAPSHOT      (63u * FFT_N)                      /* 201.6 us; capture limit is 16380 */
#define BLOCKS        (SNAPSHOT / FFT_N)
#define STRONGEST     3u
#define SPAN_HALF_MHZ 34 /* 33.75 MHz, labelled */
#define HISTORY       33
#define HEADER_ROWS   7
#define SPECTRUM_ROWS 10
#define TOAST_US      1500000
#define LONG_PRESS_US 600000

#ifndef CONFIG_ESP_SDR_OLED_SDA
#define CONFIG_ESP_SDR_OLED_SDA 5
#endif
#ifndef CONFIG_ESP_SDR_OLED_SCL
#define CONFIG_ESP_SDR_OLED_SCL 6
#endif
#ifndef CONFIG_ESP_SDR_OLED_BUTTON
#define CONFIG_ESP_SDR_OLED_BUTTON 9
#endif
#ifndef CONFIG_ESP_SDR_OLED_COLUMN_OFFSET
#define CONFIG_ESP_SDR_OLED_COLUMN_OFFSET 28
#endif

typedef enum { MODE_SPLIT, MODE_FULL, MODE_SLOW, MODE_COUNT } view_mode_t;
static const char *const mode_names[MODE_COUNT] = {"SPLIT", "FULL", "SLOW"};
static const unsigned line_period_us[MODE_COUNT] = {100000, 100000, 500000};
static const unsigned presets[] = {2437, 2462, 2480, 2412, 2400};

static i2c_master_dev_handle_t panel;
static bool present, enabled = true;
static view_mode_t mode;
static uint8_t fb[PAGES][W];

static int16_t fft[2 * FFT_N] __attribute__((aligned(16)));
static int16_t window[FFT_N];
static uint8_t position[W * BINS_PER_COL]; /* FFT output index per display bin */
static uint32_t peak[W * BINS_PER_COL];
static spectrum_dc_t dc;

static uint8_t column_db[W]; /* last completed line, 0.5 dB codes */
static uint8_t history[HISTORY][(W + 7) / 8];
static unsigned history_head, line_seq;
static int floor_q4 = -1; /* noise floor estimate, 0.5 dB << 4 */
static int64_t line_start;
static unsigned line_ffts, last_line_ffts, rotating_block;

static char toast[20];
static int64_t toast_until;
static int64_t press_start;
static bool long_fired;
static unsigned preset_index;

/* ---- SSD1306 ---------------------------------------------------------- */

static bool command(const uint8_t *bytes, size_t count) {
    uint8_t buffer[32] = {0x00};
    memcpy(buffer + 1, bytes, count);
    return i2c_master_transmit(panel, buffer, count + 1, 50) == ESP_OK;
}

static bool panel_flush(void) {
    static const uint8_t window_cmd[] = {
        0x21,     CONFIG_ESP_SDR_OLED_COLUMN_OFFSET, CONFIG_ESP_SDR_OLED_COLUMN_OFFSET + W - 1, 0x22, 0,
        PAGES - 1};
    static uint8_t data[1 + sizeof(fb)];
    if (!command(window_cmd, sizeof(window_cmd))) return false;
    data[0] = 0x40;
    memcpy(data + 1, fb, sizeof(fb));
    return i2c_master_transmit(panel, data, sizeof(data), 50) == ESP_OK;
}

static bool panel_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = CONFIG_ESP_SDR_OLED_SDA,
        .scl_io_num = CONFIG_ESP_SDR_OLED_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_config, &bus) != ESP_OK) return false;
    if (i2c_master_probe(bus, 0x3c, 50) != ESP_OK) return false;
    i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3c,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &device, &panel) != ESP_OK) return false;
    /* 72x40 "ER" module: 40-row multiplex, internal IREF, columns 28..99. */
    static const uint8_t init[] = {0xae, 0xd5, 0x80, 0xa8, H - 1, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00,
#if CONFIG_ESP_SDR_OLED_FLIP
                                   0xa0, 0xc0,
#else
                                   0xa1, 0xc8,
#endif
                                   0xda, 0x12, 0xad, 0x30, 0x81,  0x7f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6};
    if (!command(init, sizeof(init))) return false;
    memset(fb, 0, sizeof(fb));
    if (!panel_flush()) return false;
    static const uint8_t on[] = {0xaf};
    return command(on, sizeof(on));
}

/* ---- Drawing ---------------------------------------------------------- */

static void pixel(int x, int y) {
    if ((unsigned)x < W && (unsigned)y < H) fb[y >> 3][x] |= 1u << (y & 7);
}

/* 3x5 glyphs, five 3-bit rows (MSB = left). */
static uint16_t glyph(char c) {
    static const uint16_t digits[10] = {075557, 026227, 071747, 071717, 055711,
                                        074717, 074757, 071122, 075757, 075717};
    static const uint16_t letters[26] = {025755, 065656, 034443, 065556, 074647, 074644, 034553,
                                         055755, 072227, 011152, 055655, 044447, 057755, 065555,
                                         025552, 065644, 025573, 065655, 034216, 072222, 055557,
                                         055552, 055775, 055255, 055222, 071247};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    switch (c) {
    case '-': return 000700;
    case '+': return 002720;
    case '.': return 000002;
    case ':': return 002020;
    case '/': return 011244;
    default: return 0;
    }
}

static int text(int x, int y, const char *s) {
    for (; *s; s++, x += 4) {
        uint16_t g = glyph(*s);
        for (int row = 0; row < 5; row++)
            for (int col = 0; col < 3; col++)
                if (g & (1u << (14 - row * 3 - col))) pixel(x + col, y + row);
    }
    return x;
}

static int text_width(const char *s) {
    return s[0] ? (int)strlen(s) * 4 - 1 : 0;
}

static void set_toast(const char *message) {
    snprintf(toast, sizeof(toast), "%s", message);
    toast_until = esp_timer_get_time() + TOAST_US;
}

static void render(unsigned frequency_mhz, bool host_active) {
    memset(fb, 0, sizeof(fb));
    char label[20];
    if (host_active) {
        text((W - text_width("USB HOST")) / 2, 0, "USB HOST");
    } else if (esp_timer_get_time() < toast_until) {
        text((W - text_width(toast)) / 2, 0, toast);
    } else {
        snprintf(label, sizeof(label), "%u", frequency_mhz - SPAN_HALF_MHZ);
        text(0, 0, label);
        snprintf(label, sizeof(label), "%u", frequency_mhz);
        text((W - text_width(label)) / 2, 0, label);
        snprintf(label, sizeof(label), "%u", frequency_mhz + SPAN_HALF_MHZ);
        text(W - text_width(label), 0, label);
    }
    for (int x = 0; x < W; x += 9) pixel(x, 6);
    pixel(W - 1, 6);

    int waterfall_top = HEADER_ROWS;
    if (mode == MODE_SPLIT) {
        int bottom = HEADER_ROWS + SPECTRUM_ROWS - 1;
        int floor_db = floor_q4 >> 4;
        for (int x = 0; x < W; x++) {
            int height = ((int)column_db[x] - floor_db - 4) * SPECTRUM_ROWS / 50; /* 25 dB */
            if (height < 0) height = 0;
            if (height > SPECTRUM_ROWS) height = SPECTRUM_ROWS;
            for (int y = 0; y < height; y++) pixel(x, bottom - y);
        }
        waterfall_top += SPECTRUM_ROWS;
    }
    for (int y = waterfall_top; y < H; y++) {
        int age = y - waterfall_top;
        const uint8_t *row = history[(history_head + HISTORY - age) % HISTORY];
        for (int x = 0; x < W; x++)
            if (row[x >> 3] & (0x80u >> (x & 7))) pixel(x, y);
    }
}

/* ---- Signal processing ------------------------------------------------ */

static unsigned bit_reverse(unsigned value, unsigned bits) {
    unsigned result = 0;
    for (unsigned j = 0; j < bits; j++, value >>= 1) result = (result << 1) | (value & 1);
    return result;
}

/* ESP-WebSDR shows conj(I+jQ): display bin d holds FFT bin (n/2-d) mod n.
 * The radix-2 kernel leaves bins in bit-reversed order. */
static void build_tables(void) {
    for (unsigned j = 0; j < FFT_N; j++)
        window[j] = (int16_t)lrintf(16383.5f * (1 - cosf(2 * (float)M_PI * j / FFT_N)));
    for (unsigned d = 0; d < W * BINS_PER_COL; d++) {
        unsigned bin = (FFT_N / 2 - (FIRST_BIN + d)) & (FFT_N - 1);
        position[d] = bit_reverse(bin, 8);
    }
}

static void no_poll(void *context) {
    (void)context;
}

static void transform_block(const uint32_t *words) {
    for (unsigned j = 0; j < FFT_N; j++) {
        uint32_t w = words[j];
        int32_t i = (int32_t)(w << 22) >> 22, q = (int32_t)(w << 12) >> 22;
        fft[2 * j] = spectrum_sat16((i * window[j]) >> 9);
        fft[2 * j + 1] = spectrum_sat16((q * window[j]) >> 9);
    }
    spectrum_fft_poll(fft, FFT_N, dsps_fft_w_table_sc16, no_poll, NULL);
    spectrum_dc_apply(&dc, fft, FFT_N);
    for (unsigned d = 0; d < W * BINS_PER_COL; d++) {
        unsigned p = position[d];
        uint32_t power = spectrum_complex_power(fft[2 * p], fft[2 * p + 1]);
        if (power > peak[d]) peak[d] = power;
    }
    line_ffts++;
}

static void process_snapshot(const uint32_t *words, unsigned count) {
    unsigned blocks = count / FFT_N, chosen[STRONGEST + 1], picks = 0;
    uint32_t energy[BLOCKS];
    for (unsigned b = 0; b < blocks; b++) {
        uint32_t sum = 0;
        for (unsigned j = b * FFT_N; j < (b + 1) * FFT_N; j++) {
            int32_t i = (int32_t)(words[j] << 22) >> 22, q = (int32_t)(words[j] << 12) >> 22;
            sum += (uint32_t)(i * i + q * q); /* 256 * 2 * 512^2 fits */
        }
        energy[b] = sum;
    }
    for (; picks < STRONGEST && picks < blocks; picks++) {
        unsigned best = 0;
        for (unsigned b = 1; b < blocks; b++)
            if (energy[b] > energy[best]) best = b;
        chosen[picks] = best;
        energy[best] = 0;
    }
    rotating_block = (rotating_block + 1) % blocks;
    if (energy[rotating_block]) chosen[picks++] = rotating_block;
    for (unsigned k = 0; k < picks; k++) transform_block(words + chosen[k] * FFT_N);
}

static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

static void finish_line(void) {
    /* Bins next to DC carry residual LO leakage; let the column's other
     * bins speak for it. */
    unsigned dc_display = FFT_N / 2 - FIRST_BIN;
    unsigned histogram[256] = {0};
    for (unsigned x = 0; x < W; x++) {
        uint32_t best = 0;
        for (unsigned k = 0; k < BINS_PER_COL; k++) {
            unsigned d = x * BINS_PER_COL + k;
            if (d + 1 >= dc_display && d <= dc_display + 1) continue;
            if (peak[d] > best) best = peak[d];
        }
        column_db[x] = spectrum_power_code(best);
        histogram[column_db[x]]++;
    }
    memset(peak, 0, sizeof(peak));
    last_line_ffts = line_ffts;
    line_ffts = 0;

    /* 25th percentile tracks the floor even with a busy band. */
    unsigned count = 0, percentile = 0;
    while (percentile < 255 && (count += histogram[percentile]) < W / 4) percentile++;
    if (floor_q4 < 0)
        floor_q4 = percentile << 4;
    else
        floor_q4 += ((int)(percentile << 4) - floor_q4) / 2;

    /* 0..16 levels over 24 dB, starting 2 dB above the floor. */
    int floor_db = floor_q4 >> 4;
    history_head = (history_head + 1) % HISTORY;
    uint8_t *row = history[history_head];
    memset(row, 0, (W + 7) / 8);
    line_seq++;
    for (unsigned x = 0; x < W; x++) {
        int level = ((int)column_db[x] - floor_db - 4) * 16 / 48;
        if (level > bayer[line_seq & 3][x & 3]) row[x >> 3] |= 0x80u >> (x & 7);
    }
}

/* ---- Button ----------------------------------------------------------- */

static void poll_button(unsigned frequency_mhz, oled_retune_fn retune) {
    bool down = gpio_get_level(CONFIG_ESP_SDR_OLED_BUTTON) == 0;
    int64_t now = esp_timer_get_time();
    char message[20];
    if (down && !press_start) {
        press_start = now;
        long_fired = false;
    } else if (down && !long_fired && now - press_start >= LONG_PRESS_US) {
        long_fired = true;
        mode = (mode + 1) % MODE_COUNT;
        snprintf(message, sizeof(message), "VIEW %s", mode_names[mode]);
        set_toast(message);
    } else if (!down && press_start) {
        if (!long_fired && now - press_start > 30000) {
            for (unsigned j = 0; j < sizeof(presets) / sizeof(presets[0]); j++)
                if (presets[j] == frequency_mhz) preset_index = j;
            preset_index = (preset_index + 1) % (sizeof(presets) / sizeof(presets[0]));
            retune(presets[preset_index]);
            snprintf(message, sizeof(message), "%u MHZ", presets[preset_index]);
            set_toast(message);
            memset(peak, 0, sizeof(peak));
            line_ffts = 0;
            line_start = now;
        }
        press_start = 0;
    }
}

/* ---- Public ----------------------------------------------------------- */

bool oled_waterfall_init(unsigned frequency_mhz) {
    gpio_config_t button = {
        .pin_bit_mask = 1ull << CONFIG_ESP_SDR_OLED_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&button);
    build_tables();
    spectrum_fft_init();
    present = panel_init();
    if (!present) return false;
    char message[20];
    snprintf(message, sizeof(message), "ESP-SDR %u", frequency_mhz);
    set_toast(message);
    render(frequency_mhz, false);
    panel_flush();
    line_start = esp_timer_get_time();
    return true;
}

void oled_waterfall_step(unsigned frequency_mhz, bool host_active, spectrum_acquire_fn acquire,
                         oled_retune_fn retune) {
    static bool host_shown;
    if (!present || !enabled) return;
    if (host_active) {
        if (!host_shown) {
            render(frequency_mhz, true);
            panel_flush();
            host_shown = true;
        }
        line_start = esp_timer_get_time();
        return;
    }
    host_shown = false;
    poll_button(frequency_mhz, retune);

    const uint32_t *words;
    unsigned elapsed;
    if (acquire(SNAPSHOT, 0, &words, &elapsed)) process_snapshot(words, SNAPSHOT);

    int64_t now = esp_timer_get_time();
    if (now - line_start >= line_period_us[mode] && line_ffts) {
        line_start = now;
        finish_line();
        render(frequency_mhz, false);
        panel_flush();
    }
}

bool oled_waterfall_command(const char *line) {
    char reply[96];
    unsigned value;
    char extra;
    if (!strcmp(line, "OLED?")) {
        snprintf(reply, sizeof(reply), "OLED %s %s %u %d %u %u\n", present ? "PRESENT" : "ABSENT",
                 enabled ? "ON" : "OFF", (unsigned)mode, floor_q4 >> 4, line_seq, last_line_ffts);
        burst_serial_send(reply, strlen(reply));
        return true;
    }
    if (!strcmp(line, "OLEDDUMP")) {
        /* ASCII copy of the current frame buffer, for bench checks. */
        char row[W + 1];
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) row[x] = fb[y >> 3][x] & (1u << (y & 7)) ? '#' : '.';
            row[W] = '\n';
            burst_serial_send(row, sizeof(row));
        }
        snprintf(reply, sizeof(reply), "END %u\n", line_seq);
        burst_serial_send(reply, strlen(reply));
        return true;
    }
    if (!strcmp(line, "OLEDLINE")) {
        /* Last line's column levels in 0.5 dB codes plus the floor. */
        snprintf(reply, sizeof(reply), "OLEDLINE %d", floor_q4 >> 4);
        burst_serial_send(reply, strlen(reply));
        for (int x = 0; x < W; x++) {
            snprintf(reply, sizeof(reply), " %u", column_db[x]);
            burst_serial_send(reply, strlen(reply));
        }
        burst_serial_send("\n", 1);
        return true;
    }
    if (!strcmp(line, "OLED ON") || !strcmp(line, "OLED OFF")) {
        enabled = line[6] == 'N';
        if (present && !enabled) {
            memset(fb, 0, sizeof(fb));
            panel_flush();
        }
        burst_serial_send("OK\n", 3);
        return true;
    }
    if (sscanf(line, "OLEDMODE %u %c", &value, &extra) == 1 && value < MODE_COUNT) {
        mode = value;
        burst_serial_send("OK\n", 3);
        return true;
    }
    return false;
}
