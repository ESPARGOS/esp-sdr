/* Frame-buffer dump over the console UART for documentation and remote
 * checks: the host sends "SHOT\n" and receives the raw RGB565 frame. */
#include "screenshot.h"
#include <string.h>
#include "display.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void screenshot_task(void *arg) {
    (void)arg;
    char line[32];
    unsigned used = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;
        if (c != '\n' && c != '\r') {
            if (used < sizeof(line) - 1) line[used++] = (char)c;
            else used = 0;
            continue;
        }
        line[used] = '\0';
        used = 0;
        if (strcmp(line, "SHOT")) {
            if (used || line[0]) ESP_LOGI("console", "%s: %s", line, ui_command(line) ? "ok" : "unknown command");
            continue;
        }
        esp_log_level_t level = esp_log_level_get("*");
        esp_log_level_set("*", ESP_LOG_NONE);
        char header[32];
        int length = snprintf(header, sizeof(header), "SHOT %d %d\n", DISPLAY_WIDTH, DISPLAY_HEIGHT);
        uart_write_bytes(UART_NUM_0, header, length);
        /* Streaming takes seconds while the renderer keeps drawing; send a
         * consistent copy of the frame instead of a temporal smear. */
        size_t total = (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
        uint8_t *copy = heap_caps_malloc(total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        const uint8_t *fb = copy ? copy : (const uint8_t *)display_frame_buffer();
        if (copy) memcpy(copy, display_frame_buffer(), total);
        for (size_t at = 0; at < total; at += 4096) {
            size_t chunk = total - at < 4096 ? total - at : 4096;
            uart_write_bytes(UART_NUM_0, fb + at, chunk);
        }
        heap_caps_free(copy);
        uart_write_bytes(UART_NUM_0, "\nEND\n", 5);
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
        esp_log_level_set("*", level);
    }
}

void screenshot_start(void) {
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    xTaskCreatePinnedToCore(screenshot_task, "screenshot", 4096, NULL, 6, NULL, 0);
}
