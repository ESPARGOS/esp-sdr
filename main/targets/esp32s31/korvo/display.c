/* ESP32-S31-Korvo-1 with the ESP32-S3-LCD-EV-Board-SUB3: 800x480 RGB565 panel
 * on the LCD_CAM RGB interface, GT1151 capacitive touch on I2C (GPIO0/1).
 * Pins and timing follow Espressif's esp-bsp esp32_s31_korvo_1 package. */
#include "display.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt1151.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"

static const char *TAG = "display";
static esp_lcd_panel_handle_t panel;
static void *frame_buffer;

esp_lcd_panel_handle_t display_panel(void) { return panel; }
const uint16_t *display_frame_buffer(void) { return frame_buffer; }

void display_blit(int x0, int y0, int x1, int y1, const uint16_t *pixels) {
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x0, y0, x1, y1, pixels));
}

lv_display_t *display_init(void) {
    esp_lcd_rgb_panel_config_t config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .dma_burst_size = 64,
        .data_width = 16,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .de_gpio_num = 43,
        .pclk_gpio_num = 40,
        .vsync_gpio_num = 45,
        .hsync_gpio_num = 44,
        .disp_gpio_num = 38,
        .data_gpio_nums = {8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 33, 34, 35, 36},
        .timings = {
            .pclk_hz = 18000000,
            .h_res = DISPLAY_WIDTH,
            .v_res = DISPLAY_HEIGHT,
            .hsync_pulse_width = 40,
            .hsync_back_porch = 40,
            .hsync_front_porch = 48,
            .vsync_pulse_width = 23,
            .vsync_back_porch = 32,
            .vsync_front_porch = 13,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = 1,
        .num_fbs = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel, 1, &frame_buffer));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    i2c_master_bus_handle_t i2c;
    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = 1,
        .sda_io_num = 0,
        .scl_io_num = 1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &i2c));
    esp_lcd_panel_io_handle_t touch_io;
    const esp_lcd_panel_io_i2c_config_t touch_io_config = ESP_LCD_TOUCH_IO_I2C_GT1151_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c, &touch_io_config, &touch_io));
    esp_lcd_touch_handle_t touch = NULL;
    const esp_lcd_touch_config_t touch_config = {
        .x_max = DISPLAY_WIDTH,
        .y_max = DISPLAY_HEIGHT,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {.reset = 0, .interrupt = 0},
    };
    esp_err_t touch_result = esp_lcd_touch_new_i2c_gt1151(touch_io, &touch_config, &touch);
    if (touch_result != ESP_OK) ESP_LOGW(TAG, "Touch controller unavailable: %s", esp_err_to_name(touch_result));

    lvgl_port_cfg_t port = ESP_LVGL_PORT_INIT_CONFIG();
    port.task_affinity = 0;
    port.task_priority = 5;
    port.task_stack = 8192;
    port.timer_period_ms = 2;
    ESP_ERROR_CHECK(lvgl_port_init(&port));
    const lvgl_port_display_cfg_t disp_config = {
        .panel_handle = panel,
        .buffer_size = DISPLAY_WIDTH * 32,
        .double_buffer = false,
        .hres = DISPLAY_WIDTH,
        .vres = DISPLAY_HEIGHT,
        .color_format = LV_COLOR_FORMAT_RGB565,
        /* UI redraws are rare; keep internal RAM for the capture and FFT paths. */
        .flags = {.buff_spiram = true},
    };
    const lvgl_port_display_rgb_cfg_t rgb_config = {0};
    lv_display_t *display = lvgl_port_add_disp_rgb(&disp_config, &rgb_config);
    if (touch) {
        const lvgl_port_touch_cfg_t touch_port = {.disp = display, .handle = touch};
        lvgl_port_add_touch(&touch_port);
    }
    return display;
}
