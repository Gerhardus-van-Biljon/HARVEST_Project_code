#ifndef APP_WIFI_H
#define APP_WIFI_H

#include "esp_err.h"

/**
 * Initialize ESP32 Wi-Fi in Station Mode using credentials configured in menuconfig/Kconfig.
 * Prints assigned IP address upon successful connection.
 */
esp_err_t app_wifi_init_sta(void);

#endif // APP_WIFI_H
