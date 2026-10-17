# ESP32-S31-Korvo-1: on-device spectrum analyzer with the 800x480 RGB touch LCD.
idf_component_register(SRCS "app.c" "display.c" "radio.c" "analyzer.c" "plot.c" "ui.c" "logo.c" "screenshot.c" "s31_power.S"
    "../burst/s31_fft_rnd.S" "../burst/s31_unpack.S"
    "../../../common/burst_gain_table.c" "../../../common/rx_recalibration.c" "../../../common/rx_calibration_state.c"
    INCLUDE_DIRS "." "../../../common" ".."
    LDFRAGMENTS "linker.lf"
    REQUIRES esp_wifi esp_phy esp_psram esp_mm nvs_flash esp_event esp_timer
    esp_driver_gpio esp_driver_i2c esp_driver_uart esp_driver_ppa esp_lcd esp-dsp
    lvgl__lvgl espressif__esp_lvgl_port espressif__esp_lcd_touch_gt1151)
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=phy_write_gain_mem" "-Wl,--wrap=phy_set_channel_rfpll_freq"
    "-T${CMAKE_CURRENT_LIST_DIR}/sram_guard.ld")
include("../../../common/rx_calibration_link.cmake")

# Same embedded version record as the burst firmware; the exporter reads it.
idf_build_get_property(version_python PYTHON)
set(version_header "${CMAKE_CURRENT_BINARY_DIR}/generated/build_version.h")
add_custom_target(esp_sdr_build_version
    COMMAND "${version_python}" "${PROJECT_DIR}/tools/generate_version.py"
        --root "${PROJECT_DIR}" --profile "esp32s31-korvo" --output "${version_header}"
    BYPRODUCTS "${version_header}"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} esp_sdr_build_version)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
