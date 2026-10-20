idf_component_register(
    SRCS 
        "main.c"
        "app_wifi.c"
        "app_jpeg_decoder.c"
        "detector.c"
        "net_server.c"
    INCLUDE_DIRS "."
    REQUIRES 
        "nvs_flash"
        "esp_wifi"
        "esp_event"
        "esp_jpeg"
        "esp_timer"
        "lwip"
)
