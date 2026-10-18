/**
 * @file ota_update.h
 * @brief Over-The-Air (OTA) Wireless Firmware Update System for ESP32-S3.
 * 
 * Features:
 * - Dual-mode Wi-Fi: Station (connects to local Wi-Fi) and SoftAP (standalone hotspot)
 * - Interactive embedded HTTP Web Portal with drag-and-drop firmware flashing
 * - Real-time progress reporting and live robot telemetry endpoints
 * - Hardware safety interlock (disengages motors prior to flash write)
 * - Two-stage dual partition verification with automatic rollback protection
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Wi-Fi & Network Configuration Defaults
// -----------------------------------------------------------------------------
#define OTA_DEFAULT_AP_SSID          "ESP32-Robot-OTA"
#define OTA_DEFAULT_AP_PASS          "robotics2026"
#define OTA_AP_CHANNEL               1
#define OTA_AP_MAX_CONN              4
#define OTA_AP_STATIC_IP             "192.168.4.1"

#define OTA_HTTP_PORT                80
#define OTA_MAX_CHUNK_SIZE           4096

// Default station credentials (can be overridden in ota_system_init)
#ifndef OTA_DEFAULT_STA_SSID
#define OTA_DEFAULT_STA_SSID         ""
#endif
#ifndef OTA_DEFAULT_STA_PASS
#define OTA_DEFAULT_STA_PASS         ""
#endif

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
typedef struct {
    bool   sta_connected;            // True if connected to Wi-Fi router
    bool   ap_active;                // True if SoftAP hotspot is broadcasting
    char   sta_ip[16];               // Station IP address (e.g. "192.168.1.105")
    char   ap_ip[16];                // SoftAP IP address ("192.168.4.1")
    char   running_partition[16];    // Active running partition ("ota_0" or "ota_1")
    char   next_partition[16];       // Target partition for next update
    bool   is_updating;              // True while an OTA write is in progress
    size_t bytes_written;            // Bytes written in current update session
    size_t total_size;               // Total expected binary size
} ota_status_t;

// -----------------------------------------------------------------------------
// Public APIs
// -----------------------------------------------------------------------------

/**
 * @brief Initialize NVS, dual-mode Wi-Fi (Station + SoftAP), and HTTP OTA server.
 * 
 * @param sta_ssid Optional Station SSID to connect to (pass NULL or "" to skip)
 * @param sta_pass Optional Station Password (pass NULL or "" to skip)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_system_init(const char *sta_ssid, const char *sta_pass);

/**
 * @brief Confirm that the newly booted firmware is stable, cancelling automatic rollback.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_confirm_app_valid(void);

/**
 * @brief Retrieve current OTA and network status.
 * 
 * @param out_status Pointer to status structure
 */
void ota_get_status(ota_status_t *out_status);

/**
 * @brief Print network credentials, IPs, and active OTA partition to serial console.
 */
void ota_print_network_info(void);

#ifdef __cplusplus
}
#endif

