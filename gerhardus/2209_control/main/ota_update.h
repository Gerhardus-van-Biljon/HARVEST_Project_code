/*
 * wireless ota update system for esp32-s3.
 * runs an access point ("ESP32-Robot-OTA") and optional station wifi,
 * serves an embedded web page for flashing firmware .bin files directly in the browser,
 * and handles dual-partition rollback protection.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// wifi access point defaults
#define OTA_DEFAULT_AP_SSID          "ESP32-Robot-OTA"
#define OTA_DEFAULT_AP_PASS          "robotics2026"
#define OTA_AP_CHANNEL               1
#define OTA_AP_MAX_CONN              4
#define OTA_AP_STATIC_IP             "192.168.4.1"

#define OTA_HTTP_PORT                80
#define OTA_MAX_CHUNK_SIZE           4096

// station credentials if connecting to an external router
#ifndef OTA_DEFAULT_STA_SSID
#define OTA_DEFAULT_STA_SSID         ""
#endif
#ifndef OTA_DEFAULT_STA_PASS
#define OTA_DEFAULT_STA_PASS         ""
#endif

// tracks wifi connection state and current flash partition status
typedef struct {
    bool   sta_connected;            // true if connected to local router
    bool   ap_active;                // true if access point is up
    char   sta_ip[16];               // router ip (e.g. 192.168.1.50)
    char   ap_ip[16];                // access point ip (192.168.4.1)
    char   running_partition[16];    // active slot ("ota_0" or "ota_1")
    char   next_partition[16];       // destination partition for new binary
    bool   is_updating;              // true during an active flash upload
    size_t bytes_written;            // bytes written so far
    size_t total_size;               // expected total size from content-length
} ota_status_t;

// starts wifi (station + access point) and begins listening for http update requests
esp_err_t ota_system_init(const char *sta_ssid, const char *sta_pass);

// call after booting new firmware to confirm it works properly so esp-idf won't roll back
esp_err_t ota_confirm_app_valid(void);

// fetch current network state and ota partition info
void ota_get_status(ota_status_t *out_status);

// log active ip addresses and current running partition
void ota_print_network_info(void);

#ifdef __cplusplus
}
#endif
