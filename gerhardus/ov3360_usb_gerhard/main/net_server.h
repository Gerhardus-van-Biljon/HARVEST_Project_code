#ifndef NET_SERVER_H
#define NET_SERVER_H

#include "esp_err.h"

/**
 * Start the TCP streaming server task on ESP32-S3.
 */
esp_err_t net_server_start(void);

#endif // NET_SERVER_H
