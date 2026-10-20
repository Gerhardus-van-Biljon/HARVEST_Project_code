#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "protocol.h"
#include "app_jpeg_decoder.h"
#include "detector.h"
#include "net_server.h"

static const char *TAG = "NET_SERVER";

static int read_exact(int sock, void *buf, size_t len)
{
    size_t total_read = 0;
    uint8_t *ptr = (uint8_t *)buf;
    while (total_read < len) {
        int r = recv(sock, ptr + total_read, len - total_read, 0);
        if (r <= 0) {
            return r; // Error or disconnected
        }
        total_read += r;
    }
    return (int)total_read;
}

static int send_exact(int sock, const void *buf, size_t len)
{
    size_t total_sent = 0;
    const uint8_t *ptr = (const uint8_t *)buf;
    while (total_sent < len) {
        int s = send(sock, ptr + total_sent, len - total_sent, 0);
        if (s <= 0) {
            return s; // Error or disconnected
        }
        total_sent += s;
    }
    return (int)total_sent;
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CONFIG_TCP_SERVER_PORT);

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP Server listening on port %d...", CONFIG_TCP_SERVER_PORT);

    uint8_t *jpeg_buf = NULL;
    size_t jpeg_buf_capacity = 0;

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Disable Nagle's algorithm for low-latency transmission
        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        struct timeval tv_timeout = { .tv_sec = 10, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv_timeout, sizeof(tv_timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv_timeout, sizeof(tv_timeout));

        inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Client connected from: %s", addr_str);

        while (1) {
            int64_t t_start_total = esp_timer_get_time();

            // 1. Read Frame Header
            frame_header_t header;
            int64_t t0 = esp_timer_get_time();
            int ret = read_exact(sock, &header, sizeof(header));
            if (ret <= 0) {
                ESP_LOGW(TAG, "Client disconnected or header read error");
                break;
            }

            if (header.magic != FRAME_HEADER_MAGIC) {
                ESP_LOGE(TAG, "Invalid frame header magic: 0x%08X", (unsigned int)header.magic);
                break;
            }

            // Ensure JPEG buffer capacity in PSRAM
            if (header.payload_len > jpeg_buf_capacity) {
                if (jpeg_buf) {
                    heap_caps_free(jpeg_buf);
                }
                jpeg_buf_capacity = header.payload_len + 8192; // Extra padding
                jpeg_buf = (uint8_t *)heap_caps_malloc(jpeg_buf_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!jpeg_buf) {
                    jpeg_buf = (uint8_t *)malloc(jpeg_buf_capacity);
                }
                if (!jpeg_buf) {
                    ESP_LOGE(TAG, "Failed to allocate %d bytes for JPEG buffer", jpeg_buf_capacity);
                    break;
                }
            }

            // Read JPEG Payload
            ret = read_exact(sock, jpeg_buf, header.payload_len);
            if (ret <= 0) {
                ESP_LOGE(TAG, "Failed to read full JPEG payload (%d bytes)", header.payload_len);
                break;
            }
            int64_t t1 = esp_timer_get_time();
            uint32_t recv_time_us = (uint32_t)(t1 - t0);

            // 2. Decode JPEG
            decoded_image_t img = {0};
            int64_t t2 = esp_timer_get_time();
            esp_err_t dec_res = jpeg_decode_to_rgb888(jpeg_buf, header.payload_len, &img);
            int64_t t3 = esp_timer_get_time();
            uint32_t decode_time_us = (uint32_t)(t3 - t2);

            detection_box_t boxes[MAX_DETECTIONS];
            uint16_t num_detections = 0;
            uint32_t inference_time_us = 0;

            if (dec_res == ESP_OK) {
                // 3. Object Detection Inference
                int64_t t4 = esp_timer_get_time();
                detector_run(&img, boxes, MAX_DETECTIONS, &num_detections);
                int64_t t5 = esp_timer_get_time();
                inference_time_us = (uint32_t)(t5 - t4);

                jpeg_decoder_free_image(&img);
            } else {
                ESP_LOGE(TAG, "Skipping detection due to JPEG decode failure");
            }

            // 4. Send Response Header + Results (+ Frame if Stage 2)
            int64_t t6 = esp_timer_get_time();
            response_header_t resp = {
                .magic = RESPONSE_HEADER_MAGIC,
                .frame_id = header.frame_id,
                .stage_mode = header.stage_mode,
                .num_detections = num_detections,
                .echoed_frame_len = (header.stage_mode == 2) ? header.payload_len : 0,
                .timing = {
                    .recv_time_us = recv_time_us,
                    .decode_time_us = decode_time_us,
                    .inference_time_us = inference_time_us,
                    .send_time_us = 0,
                    .total_esp_time_us = 0
                }
            };

            // Buffer response header + bounding boxes into stack buffer
            uint8_t resp_hdr_buf[256];
            size_t boxes_len = num_detections * sizeof(detection_box_t);
            size_t hdr_boxes_len = sizeof(resp) + boxes_len;

            memcpy(resp_hdr_buf, &resp, sizeof(resp));
            if (boxes_len > 0) {
                memcpy(resp_hdr_buf + sizeof(resp), boxes, boxes_len);
            }

            ret = send_exact(sock, resp_hdr_buf, hdr_boxes_len);

            // If Stage 2: Echo back JPEG frame
            if (ret > 0 && header.stage_mode == 2 && header.payload_len > 0) {
                ret = send_exact(sock, jpeg_buf, header.payload_len);
            }

            if (ret <= 0) {
                ESP_LOGE(TAG, "Failed to send response payload to client");
                break;
            }

            int64_t t7 = esp_timer_get_time();
            uint32_t send_time_us = (uint32_t)(t7 - t6);
            uint32_t total_esp_time_us = (uint32_t)(t7 - t_start_total);

            // Log benchmarking profile over serial
            ESP_LOGI(TAG, "[v1.2.0] Frame #%d [Stage %d] | Recv: %.2fms | Dec: %.2fms | Infer: %.2fms | Send: %.2fms | Total: %.2fms | Detections: %d",
                     header.frame_id, header.stage_mode,
                     recv_time_us / 1000.0f,
                     decode_time_us / 1000.0f,
                     inference_time_us / 1000.0f,
                     send_time_us / 1000.0f,
                     total_esp_time_us / 1000.0f,
                     num_detections);
        }

        close(sock);
        ESP_LOGI(TAG, "Client disconnected, waiting for new connection...");
    }

    if (jpeg_buf) {
        heap_caps_free(jpeg_buf);
    }
    close(listen_sock);
    vTaskDelete(NULL);
}

esp_err_t net_server_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TCP server task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
