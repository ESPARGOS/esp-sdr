#include "burst_gpio.h"
#include "burst_serial.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_private/esp_gpio_reserve.h"
#include "soc/soc_caps.h"
#include "soc/io_mux_reg.h"
#if CONFIG_IDF_TARGET_ESP32S2
#include "soc/usb_pins.h"
#endif

#define PIN(n)            (UINT64_C(1) << (n))
#define PINS(first, last) ((PIN((last) + 1) - 1) & ~(PIN(first) - 1))
static uint64_t available;
static char states[SOC_GPIO_PIN_COUNT];

static uint64_t excluded_pins(void) {
    /* Conservatively exclude dedicated memory pads, including optional PSRAM
     * and VDD_SPI even when that memory is not enabled in this firmware.
     * The SDK reservation mask below also covers remapped memory pins. */
#if CONFIG_IDF_TARGET_ESP32
    uint64_t mask = PINS(6, 11) | PINS(16, 17);
#elif CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3
    uint64_t mask = PINS(11, 17);
#elif CONFIG_IDF_TARGET_ESP32C5
    uint64_t mask = PINS(15, 22);
#elif CONFIG_IDF_TARGET_ESP32C6
    uint64_t mask = PINS(24, 30);
#elif CONFIG_IDF_TARGET_ESP32C61
    uint64_t mask = PINS(14, 21);
#elif CONFIG_IDF_TARGET_ESP32H2
    uint64_t mask = PINS(15, 21);
#elif CONFIG_IDF_TARGET_ESP32S2
    uint64_t mask = PINS(26, 32);
#elif CONFIG_IDF_TARGET_ESP32S3
    uint64_t mask = PINS(26, 37);
#elif CONFIG_IDF_TARGET_ESP32S31
    uint64_t mask = PINS(26, 32);
#else
#error Define memory pin exclusions for this target before enabling GPIO control
#endif
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    mask |= PIN(USB_INT_PHY0_DM_GPIO_NUM) | PIN(USB_INT_PHY0_DP_GPIO_NUM);
#elif CONFIG_IDF_TARGET_ESP32S2
    mask |= PIN(USBPHY_DM_NUM) | PIN(USBPHY_DP_NUM);
#endif
#if CONFIG_ESP_SDR_UART_ENABLED
    mask |= PIN(CONFIG_ESP_SDR_UART_TX_PIN) | PIN(CONFIG_ESP_SDR_UART_RX_PIN);
#endif
#if CONFIG_ESP_SDR_DINMETER
    /* LCD 4-9, buzzer 3, RTC I2C 11/12, encoder 40-42, power hold 46. */
    mask |= PINS(3, 9) | PINS(11, 12) | PINS(40, 42) | PIN(46);
#endif
#if CONFIG_ESP_SDR_OLED_ENABLED
    mask |= PIN(CONFIG_ESP_SDR_OLED_SDA) | PIN(CONFIG_ESP_SDR_OLED_SCL) | PIN(CONFIG_ESP_SDR_OLED_BUTTON);
#endif
    return mask;
}

void burst_gpio_init(void) {
    available = SOC_GPIO_VALID_OUTPUT_GPIO_MASK & ~excluded_pins();
    for (unsigned pin = 0; pin < SOC_GPIO_PIN_COUNT; ++pin) {
        uint64_t bit = PIN(pin);
        if (!(available & bit)) continue;
        if (esp_gpio_is_reserved(bit)) {
            available &= ~bit;
            continue;
        }
        /* DISABLE disconnects both input and output; no internal pulls. Do
         * this only at boot, never as a side effect of discovery/reconnect. */
        const gpio_config_t config = {
            .pin_bit_mask = bit,
            .mode = GPIO_MODE_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
        esp_gpio_reserve(bit);
        states[pin] = 'Z';
    }
}

static void reply(const char *s) {
    burst_serial_send(s, strlen(s));
}

bool burst_gpio_command(const char *line) {
    if (!strcmp(line, "GPIO?")) {
        /* At most 64 entries of " 63:Z", plus prefix/newline/NUL. */
        char response[6 + 64 * 5];
        size_t used = (size_t)snprintf(response, sizeof(response), "GPIO");
        for (unsigned pin = 0; pin < SOC_GPIO_PIN_COUNT; ++pin)
            if (available & PIN(pin))
                used +=
                    (size_t)snprintf(response + used, sizeof(response) - used, " %u:%c", pin, states[pin]);
        response[used++] = '\n';
        response[used] = '\0';
        reply(response);
        return true;
    }
    if (strcmp(line, "GPIO") && strncmp(line, "GPIO ", 5)) return false;
    unsigned pin;
    char state, extra, canonical[32];
    if (sscanf(line, "GPIO %u %c %c", &pin, &state, &extra) != 2 || pin >= SOC_GPIO_PIN_COUNT ||
        !(available & PIN(pin)) || (state != 'Z' && state != '0' && state != '1')) {
        reply("ERR gpio_args\n");
        return true;
    }
    snprintf(canonical, sizeof(canonical), "GPIO %u %c", pin, state);
    if (strcmp(line, canonical)) {
        reply("ERR gpio_args\n");
        return true;
    }
    /* Preload the latch before enabling output, avoiding a low pulse when
     * moving from Z to 1. The pin stays in GPIO mode after initialization. */
    esp_err_t err = ESP_OK;
    if (state != 'Z') err = gpio_set_level(pin, state == '1');
    if (err == ESP_OK) err = gpio_set_direction(pin, state == 'Z' ? GPIO_MODE_DISABLE : GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        reply("ERR gpio_io\n");
        return true;
    }
    states[pin] = state;
    char response[32];
    snprintf(response, sizeof(response), "OK GPIO %u %c\n", pin, state);
    reply(response);
    return true;
}
