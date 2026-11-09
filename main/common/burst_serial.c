/* The burst protocol is shared by native USB Serial/JTAG and UART0. */
#include "burst_serial.h"
#include "burst_gpio.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include "hal/usb_serial_jtag_ll.h"
#endif
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32S2
#include "esp_private/usb_console.h"
#endif

#ifndef CONFIG_ESP_SDR_UART_BAUD
#define CONFIG_ESP_SDR_UART_BAUD 2000000
#endif

#define COMMAND_SIZE 128
static burst_serial_port_t active_port = BURST_SERIAL_USB;
static unsigned next_port;
static unsigned uart_baud = CONFIG_ESP_SDR_UART_BAUD;
static struct {
    char line[COMMAND_SIZE];
    size_t used;
    bool overflow;
    int64_t last_byte;
} input[BURST_SERIAL_COUNT];

/* Transport commands are consumed here for every receiver backend. The ACK
 * is completely transmitted at the old rate before changing UART0. The rate
 * is session-only; every boot starts with CONFIG_ESP_SDR_UART_BAUD. */
static bool baud_command(const char *line) {
    if (strcmp(line, "BAUD?") && strcmp(line, "BAUD") && strncmp(line, "BAUD ", 5)) return false;
#if CONFIG_ESP_SDR_UART_ENABLED
    if (active_port == BURST_SERIAL_UART) {
        char response[32];
        if (!strcmp(line, "BAUD?")) {
            int length = snprintf(response, sizeof(response), "BAUD %u\n", uart_baud);
            burst_serial_send(response, length);
            return true;
        }
        unsigned baud = !strcmp(line, "BAUD 1000000") ? 1000000 : !strcmp(line, "BAUD 2000000") ? 2000000 : 0;
        if (!baud) {
            burst_serial_send("ERR baud_args\n", sizeof("ERR baud_args\n") - 1);
            return true;
        }
        int length = snprintf(response, sizeof(response), "OK BAUD %u\n", baud);
        if (!burst_serial_send(response, length) ||
            uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(1000)) != ESP_OK ||
            uart_set_baudrate(UART_NUM_0, baud) != ESP_OK) {
            return true;
        }
        uart_baud = baud;
        uart_flush_input(UART_NUM_0);
        memset(&input[BURST_SERIAL_UART], 0, sizeof(input[BURST_SERIAL_UART]));
        return true;
    }
#endif
    burst_serial_send("ERR baud_transport\n", sizeof("ERR baud_transport\n") - 1);
    return true;
}

#if CONFIG_IDF_TARGET_ESP32S2
static void usb_ready(void *arg) {
    (void)arg;
}
#endif
void burst_serial_init(void) {
#if CONFIG_IDF_TARGET_ESP32S2
    ESP_ERROR_CHECK(esp_usb_console_set_cb(usb_ready, usb_ready, NULL));
#endif
#if CONFIG_ESP_SDR_UART_ENABLED
    uart_baud = CONFIG_ESP_SDR_UART_BAUD;
    const uart_config_t config = {
        .baud_rate = uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, CONFIG_ESP_SDR_UART_TX_PIN, CONFIG_ESP_SDR_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* Interrupt-driven RX keeps uploads buffered while the command is parsed.
     * TX uses the FIFO directly so all waits have a bounded deadline. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 8192, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_flush_input(UART_NUM_0));
#endif
    burst_gpio_init();
}

burst_serial_port_t burst_serial_port(void) {
    return active_port;
}
unsigned burst_serial_baud(void) {
    return active_port == BURST_SERIAL_UART ? uart_baud : 0;
}

static int read_port(burst_serial_port_t port, void *buffer, size_t size) {
    if (port == BURST_SERIAL_UART) {
#if CONFIG_ESP_SDR_UART_ENABLED
        return uart_read_bytes(UART_NUM_0, buffer, size, 0);
#else
        return 0;
#endif
    }
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    return usb_serial_jtag_ll_read_rxfifo(buffer, size > 64 ? 64 : size);
#elif CONFIG_IDF_TARGET_ESP32S2
    return esp_usb_console_read_buf(buffer, size);
#else
    return 0;
#endif
}

bool burst_serial_stop_requested(void) {
    char c;
    bool stopped = false;
    for (unsigned j = 0; j < 128 && read_port(active_port, &c, 1) == 1; j++) {
        stopped = true;
        if (c == '\n') break;
    }
    return stopped;
}

int burst_serial_poll_line(char *line, size_t capacity) {
    for (unsigned j = 0; j < BURST_SERIAL_COUNT; ++j) {
        unsigned port = (next_port + j) % BURST_SERIAL_COUNT;
        int64_t now = esp_timer_get_time();
        if (now - input[port].last_byte > 3000000) {
            input[port].used = 0;
            input[port].overflow = false;
        }
        /* Bound polling so an unterminated command cannot starve the peer. */
        for (unsigned k = 0; k < COMMAND_SIZE; ++k) {
            char ch;
            if (read_port(port, &ch, 1) != 1) break;
            input[port].last_byte = now;
            if (ch == '\r') continue;
            if (ch != '\n') {
                if (input[port].used < COMMAND_SIZE - 1)
                    input[port].line[input[port].used++] = ch;
                else
                    input[port].overflow = true;
                continue;
            }
            size_t used = input[port].used;
            bool overflow = input[port].overflow;
            input[port].used = 0;
            input[port].overflow = false;
            if (!used && !overflow) continue;
            active_port = port;
            next_port = (port + 1) % BURST_SERIAL_COUNT;
            if (overflow || used >= capacity) return -1;
            memcpy(line, input[port].line, used);
            line[used] = '\0';
            if (baud_command(line)) return 0;
            return 1;
        }
    }
    return 0;
}

static int64_t transfer_deadline(size_t size) {
    /* 8N1 needs ten wire bits per byte. Leave two seconds for host scheduling,
     * including full frames at a deliberately reduced UART baud rate. */
    int64_t timeout =
        active_port == BURST_SERIAL_UART ? 2000000 + (int64_t)size * 10000000 / uart_baud : 3000000;
    return esp_timer_get_time() + timeout;
}

/* S2 ROM CDC writes: 256 bytes per call queue several USB packets at once.
 * This doubles throughput over 64-byte writes; larger chunks gain nothing. */
#define S2_USB_CHUNK 256
size_t burst_serial_try_send(const void *data, size_t size) {
    const uint8_t *p = data;
    size_t done = 0;
    while (done < size) {
        int sent = 0;
        if (active_port == BURST_SERIAL_UART) {
            sent = uart_tx_chars(UART_NUM_0, (const char *)p + done, size - done > 128 ? 128 : size - done);
        } else {
#if SOC_USB_SERIAL_JTAG_SUPPORTED
            if (!usb_serial_jtag_ll_txfifo_writable()) break;
            sent = usb_serial_jtag_ll_write_txfifo(p + done, size - done > 64 ? 64 : size - done);
            usb_serial_jtag_ll_txfifo_flush();
#elif CONFIG_IDF_TARGET_ESP32S2
            sent = esp_usb_console_write_direct(p + done,
                                                size - done > S2_USB_CHUNK ? S2_USB_CHUNK : size - done);
#endif
        }
        if (sent <= 0) break;
        done += (unsigned)sent;
    }
    return done;
}

bool IRAM_ATTR burst_serial_send(const void *data, size_t size) {
    const uint8_t *p = data;
    size_t original = size;
    int64_t deadline = transfer_deadline(size);
    while (size) {
        if (esp_timer_get_time() >= deadline) return false;
        int sent;
        if (active_port == BURST_SERIAL_UART) {
            sent = uart_tx_chars(UART_NUM_0, (const char *)p, size > 128 ? 128 : size);
            if (sent < 0) return false;
            if (!sent) vTaskDelay(1);
        } else {
#if SOC_USB_SERIAL_JTAG_SUPPORTED
            if (!usb_serial_jtag_ll_txfifo_writable()) continue;
            sent = usb_serial_jtag_ll_write_txfifo(p, size > 64 ? 64 : size);
            usb_serial_jtag_ll_txfifo_flush();
#elif CONFIG_IDF_TARGET_ESP32S2

            sent = esp_usb_console_write_direct(p, size > S2_USB_CHUNK ? S2_USB_CHUNK : size);
            if (sent < 0) return false;
            if (!sent) taskYIELD();
#else
            return false;
#endif
        }
        p += sent;
        size -= sent;
    }
#if SOC_USB_SERIAL_JTAG_SUPPORTED
    if (active_port == BURST_SERIAL_USB && original && original % 64 == 0) {
        while (!usb_serial_jtag_ll_txfifo_writable())
            if (esp_timer_get_time() >= deadline) return false;
        usb_serial_jtag_ll_txfifo_flush();
    }
#else
    (void)original;
#endif
    return true;
}

static void serve_reply(const char *text) {
    (void)burst_serial_send(text, strlen(text));
}

static void (*serve_idle)(bool host_active);
void burst_serial_set_idle(void (*idle)(bool host_active)) {
    serve_idle = idle;
}

static bool (*serve_passive)(const char *line);
void burst_serial_set_passive(bool (*passive)(const char *line)) {
    serve_passive = passive;
}

void burst_serial_serve(void (*handle)(char *line)) {
    char line[128];
    int owner = -1;
    int64_t lease_deadline = 0;
    for (;;) {
        if (esp_timer_get_time() >= lease_deadline) owner = -1;
        int status = burst_serial_poll_line(line, sizeof(line));
        if (!status) {
            if (serve_idle) serve_idle(owner >= 0);
            vTaskDelay(1);
            continue;
        }
        if (status > 0 && serve_passive && serve_passive(line)) continue;
        int port = burst_serial_port();
        if (owner >= 0 && owner != port) {
            serve_reply("ERR busy\n");
            continue;
        }
        if (status < 0) {
            serve_reply("ERR command_length\n");
            continue;
        }
        owner = port;
        if (serve_idle) serve_idle(true);
        if (!strcmp(line, "RELEASE")) {
            serve_reply("OK\n");
            owner = -1;
        } else {
            handle(line);
        }
        /* Ownership covers the entire binary transaction. */
        lease_deadline = esp_timer_get_time() + 5000000;
    }
}
