#ifndef APP_JPEG_DECODER_H
#define APP_JPEG_DECODER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef struct {
    uint8_t *rgb888_buf;  // Pointer to output RGB888 pixel buffer (allocated in PSRAM)
    size_t width;         // Decoded image width
    size_t height;        // Decoded image height
    size_t buf_size;      // Total size of rgb888_buf in bytes
} decoded_image_t;

esp_err_t app_jpeg_decoder_init(void);
esp_err_t jpeg_decode_to_rgb888(const uint8_t *jpeg_data, size_t jpeg_len, decoded_image_t *out_img);
void jpeg_decoder_free_image(decoded_image_t *out_img);

#endif // APP_JPEG_DECODER_H
