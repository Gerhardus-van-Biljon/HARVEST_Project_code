#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "app_wifi.h"
#include "app_jpeg_decoder.h"
#include "detector.h"
#include "net_server.h"

#define FIRMWARE_VERSION "v1.2.0-SINGLE-BUFFER"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, " ESP32-S3 Wi-Fi Object Detection Test Rig");
    ESP_LOGI(TAG, " Firmware Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, " Build Timestamp:  %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, " Board: ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM)");
    ESP_LOGI(TAG, "=================================================");

    // Initialize Non-Volatile Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi Station
    ESP_LOGI(TAG, "[1/4] Initializing Wi-Fi Station...");
    if (app_wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi connection failed! Halting.");
        return;
    }

    // Initialize JPEG Decoder
    ESP_LOGI(TAG, "[2/4] Initializing JPEG Decoder...");
    ESP_ERROR_CHECK(app_jpeg_decoder_init());

    // Initialize Object Detector
    ESP_LOGI(TAG, "[3/4] Initializing Object Detector...");
    ESP_ERROR_CHECK(detector_init());

    // Start TCP Streaming Server
    ESP_LOGI(TAG, "[4/4] Starting TCP Streaming Server...");
    ESP_ERROR_CHECK(net_server_start());

    ESP_LOGI(TAG, "Test Rig Firmware ready and waiting for PC client connections.");
}
