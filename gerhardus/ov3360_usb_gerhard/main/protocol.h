#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define FRAME_HEADER_MAGIC    0x53334644  // ASCII "S3FD" (ESP32 Frame Data)
#define RESPONSE_HEADER_MAGIC 0x53335245  // ASCII "S3RE" (ESP32 Response)

#define MAX_DETECTIONS 10

#pragma pack(push, 1)

/**
 * Frame Header sent from PC -> ESP32-S3
 */
typedef struct {
    uint32_t magic;         // FRAME_HEADER_MAGIC ("S3FD")
    uint16_t frame_id;      // Incremental frame sequence counter
    uint16_t stage_mode;    // 1 = Stage 1 (results only), 2 = Stage 2 (results + echo frame)
    uint32_t image_width;   // Frame width (e.g., 320)
    uint32_t image_height;  // Frame height (e.g., 240)
    uint32_t payload_len;   // Size of the encoded JPEG frame in bytes
} frame_header_t;

/**
 * Bounding Box representation for detected objects
 */
typedef struct {
    uint16_t class_id;      // Class identifier (e.g., 0 = Person, 1 = Face, 2 = Target Object)
    float score;            // Confidence score (0.0 to 1.0)
    uint16_t x;             // Bounding box left (pixels)
    uint16_t y;             // Bounding box top (pixels)
    uint16_t w;             // Bounding box width (pixels)
    uint16_t h;             // Bounding box height (pixels)
} detection_box_t;

/**
 * Detailed latency breakdown in microseconds sent back to PC
 */
typedef struct {
    uint32_t recv_time_us;     // Time spent receiving TCP payload over Wi-Fi
    uint32_t decode_time_us;   // Time spent decompressing JPEG to RGB
    uint32_t inference_time_us;// Time spent running object detection inference
    uint32_t send_time_us;     // Time spent transmitting response back over Wi-Fi
    uint32_t total_esp_time_us;// Total processing latency on ESP32-S3
} latency_report_t;

/**
 * Detection Response Header sent from ESP32-S3 -> PC
 */
typedef struct {
    uint32_t magic;            // RESPONSE_HEADER_MAGIC ("S3RE")
    uint16_t frame_id;         // Echoed frame ID matching request
    uint16_t stage_mode;       // Stage mode processed
    uint16_t num_detections;   // Number of detected bounding boxes following
    uint32_t echoed_frame_len; // Length of JPEG frame echoed back (0 if Stage 1)
    latency_report_t timing;   // Profiling timing breakdown
} response_header_t;

#pragma pack(pop)

#endif // PROTOCOL_H
