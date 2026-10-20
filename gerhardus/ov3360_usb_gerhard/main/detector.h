#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "protocol.h"
#include "app_jpeg_decoder.h"

/**
 * Initialize object detector (loads model weights, allocates tensor memory in PSRAM).
 */
esp_err_t detector_init(void);

/**
 * Run object detection on an RGB888 image buffer.
 * 
 * @param image Pointer to decoded image structure
 * @param out_boxes Array of detection_box_t to fill with detected bounding boxes
 * @param max_boxes Capacity of out_boxes array
 * @param out_count Pointer to store number of actual detected objects
 * @return ESP_OK on success
 */
esp_err_t detector_run(const decoded_image_t *image, detection_box_t *out_boxes, uint16_t max_boxes, uint16_t *out_count);

/**
 * Deinitialize detector and release PSRAM tensor allocations.
 */
void detector_deinit(void);

#endif // DETECTOR_H
