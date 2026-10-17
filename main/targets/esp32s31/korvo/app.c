/* ESP32-S31-Korvo-1: ESP-SDR as a stand-alone touch spectrum analyzer. */
#include "analyzer.h"
#include "build_version.h"
#include "display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "plot.h"
#include "radio.h"
#include "screenshot.h"
#include "ui.h"

/* Embedded for the web exporter, which reads it from the binary. */
static const char version_record[] = ESP_SDR_VERSION_RECORD;

void app_main(void) {
    ESP_LOGI("app", "%s", version_record);
    radio_init();
    lv_display_t *display = display_init();
    plot_init();
    analyzer_start();
    ui_init(display);
    screenshot_start();
    ESP_LOGI("app", "ready: internal heap %u KiB free (largest %u KiB), PSRAM %u KiB free",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
}
