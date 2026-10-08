idf_component_register(SRCS "receiver.c" "dc.c" "control.c" "network.c" "usb.c"
    "../../../common/burst_gain_table.c" "../../../common/rx_recalibration.c" "../../../common/rx_calibration_state.c"
    INCLUDE_DIRS "." "../../../common" ".."
    REQUIRES esp_wifi esp_phy esp_psram esp_mm nvs_flash esp_event esp_timer esp_driver_parlio
    esp_driver_gpio esp_eth esp_netif esp_http_server espressif__esp_tinyusb espressif__cjson
    EMBED_TXTFILES "index.html" "logo.svg")
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=phy_write_gain_mem" "-Wl,--wrap=phy_set_channel_rfpll_freq")

include("../../../common/rx_calibration_link.cmake")
