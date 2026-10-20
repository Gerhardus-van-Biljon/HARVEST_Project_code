#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "jpeg_decoder.h" // Includes component espressif__esp_jpeg's jpeg_decoder.h
#include "app_jpeg_decoder.h"

static const char *TAG = "APP_JPEG_DEC";

esp_err_t app_jpeg_decoder_init(void)
{
    ESP_LOGI(TAG, "JPEG Decoder initialized (using espressif/esp_jpeg component v1.3.1)");
    return ESP_OK;
}

esp_err_t jpeg_decode_to_rgb888(const uint8_t *jpeg_data, size_t jpeg_len, decoded_image_t *out_img)
{
    if (!jpeg_data || jpeg_len == 0 || !out_img) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg_data,
        .indata_size = jpeg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_0,
    };

    esp_jpeg_image_output_t out_info;
    esp_err_t ret = esp_jpeg_get_image_info(&jpeg_cfg, &out_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse JPEG info (err: 0x%x)", ret);
        return ret;
    }

    out_img->width = out_info.width;
    out_img->height = out_info.height;
    out_img->buf_size = out_info.output_len;

    // Allocate output buffer in PSRAM
    out_img->rgb888_buf = (uint8_t *)heap_caps_malloc(out_img->buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_img->rgb888_buf) {
        out_img->rgb888_buf = (uint8_t *)malloc(out_img->buf_size);
    }

    if (!out_img->rgb888_buf) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for RGB888 buffer", out_img->buf_size);
        return ESP_ERR_NO_MEM;
    }

    jpeg_cfg.outbuf = out_img->rgb888_buf;
    jpeg_cfg.outbuf_size = out_img->buf_size;

    ret = esp_jpeg_decode(&jpeg_cfg, &out_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode process failed (err: 0x%x)", ret);
        jpeg_decoder_free_image(out_img);
        return ret;
    }

    return ESP_OK;
}

void jpeg_decoder_free_image(decoded_image_t *out_img)
{
    if (out_img && out_img->rgb888_buf) {
        heap_caps_free(out_img->rgb888_buf);
        out_img->rgb888_buf = NULL;
        out_img->width = 0;
        out_img->height = 0;
        out_img->buf_size = 0;
    }
}
