#pragma once
#include <stdbool.h>
#include <stdint.h>

#define RADIO_SNAPSHOT_SAMPLES 16380u
#define RADIO_RATE_COUNT 6u

typedef struct {
    unsigned frequency_mhz;   /* LO frequency, RX_FREQ_MIN..RX_FREQ_MAX */
    bool agc;                 /* hardware AGC, otherwise the manual index */
    unsigned gain;            /* manual PHY gain table index */
    unsigned bandwidth_mhz;   /* analog filter, 0 = widest capacitor code */
} radio_settings_t;

/* Wi-Fi/PHY bring-up in RF test mode and the TCM dump engine. */
void radio_init(void);
unsigned radio_gain_max(void);
unsigned radio_rate_hz(unsigned code);
/* Apply receiver settings; serialized with snapshots. */
bool radio_apply(const radio_settings_t *settings);
/* One snapshot of n IQ10 words (I bits 0..9, Q bits 10..19) at rate code 0..5.
 * *words stays valid until the next snapshot. */
bool radio_snapshot(unsigned n, unsigned rate_code, const uint32_t **words, unsigned *elapsed_us);
unsigned radio_snapshot_failures(void);
