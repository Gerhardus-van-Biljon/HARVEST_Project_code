idf_component_register(SRCS "ota_update.c" "as5600.c" "tmc2209.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio esp_driver_uart esp_driver_i2c esp_adc esp_wifi esp_http_server app_update nvs_flash esp_netif esp_event esp_timer)
