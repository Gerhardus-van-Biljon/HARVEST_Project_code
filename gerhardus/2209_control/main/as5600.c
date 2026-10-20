/**
 * @file as5600.c
 * @brief AS5600 Magnetic Rotary Encoder & TCA9548A Multiplexer Implementation.
 */

#include "as5600.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

static const char *TAG = "AS5600";

// Global encoder state array
as5600_encoder_t encoders[AS5600_NUM_ENCODERS];

// I2C Master Bus & Device Handles
static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_tca_handle = NULL;
static i2c_master_dev_handle_t s_as5600_handle = NULL;
static int8_t s_active_mux_ch = -1;

// -----------------------------------------------------------------------------
// I2C Master Bus Initialization (ESP-IDF v6.0 API)
// -----------------------------------------------------------------------------
esp_err_t as5600_i2c_bus_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz)
{
    if (s_bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing I2C Master Bus on SDA=GPIO%d, SCL=GPIO%d @ %lu Hz",
             sda_pin, scl_pin, (unsigned long)clk_speed_hz);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return err;
    }

    // 1. Register TCA9548A multiplexer device handle (Address 0x70)
    i2c_device_config_t tca_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548A_DEFAULT_ADDR,
        .scl_speed_hz = clk_speed_hz,
    };
    err = i2c_master_bus_add_device(s_bus_handle, &tca_cfg, &s_tca_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA9548A device: %s", esp_err_to_name(err));
        return err;
    }

    // 2. Register AS5600 encoder device handle (Address 0x36)
    i2c_device_config_t as5600_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = clk_speed_hz,
    };
    err = i2c_master_bus_add_device(s_bus_handle, &as5600_cfg, &s_as5600_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AS5600 device: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C devices (TCA9548A @ 0x%02X, AS5600 @ 0x%02X) registered successfully",
             TCA9548A_DEFAULT_ADDR, AS5600_I2C_ADDR);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// TCA9548A Channel Multiplexing
// -----------------------------------------------------------------------------
esp_err_t tca9548a_select_channel(uint8_t channel)
{
    if (s_tca_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_active_mux_ch == (int8_t)channel) {
        return ESP_OK; // Already selected
    }

    uint8_t control_byte = (uint8_t)(1u << channel);
    esp_err_t err = i2c_master_transmit(s_tca_handle, &control_byte, 1, AS5600_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        s_active_mux_ch = (int8_t)channel;
    } else {
        s_active_mux_ch = -1;
    }
    return err;
}

esp_err_t tca9548a_disable_all(void)
{
    if (s_tca_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t control_byte = 0x00;
    esp_err_t err = i2c_master_transmit(s_tca_handle, &control_byte, 1, AS5600_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        s_active_mux_ch = -1;
    }
    return err;
}

// -----------------------------------------------------------------------------
// Low-Level AS5600 Register Access
// -----------------------------------------------------------------------------
static esp_err_t as5600_read_reg_u8(uint8_t motor_id, uint8_t reg, uint8_t *out_val)
{
    if (motor_id >= AS5600_NUM_ENCODERS || out_val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Switch multiplexer channel if configured
    int8_t ch = encoders[motor_id].mux_channel;
    if (ch >= 0) {
        esp_err_t m_err = tca9548a_select_channel((uint8_t)ch);
        if (m_err != ESP_OK) return m_err;
    }

    uint8_t reg_addr = reg;
    return i2c_master_transmit_receive(s_as5600_handle, &reg_addr, 1, out_val, 1, AS5600_I2C_TIMEOUT_MS);
}

static esp_err_t as5600_read_reg_u16(uint8_t motor_id, uint8_t reg_msb, uint16_t *out_val)
{
    if (motor_id >= AS5600_NUM_ENCODERS || out_val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Switch multiplexer channel if configured
    int8_t ch = encoders[motor_id].mux_channel;
    if (ch >= 0) {
        esp_err_t m_err = tca9548a_select_channel((uint8_t)ch);
        if (m_err != ESP_OK) return m_err;
    }

    uint8_t reg_addr = reg_msb;
    uint8_t data[2] = {0, 0};
    esp_err_t err = i2c_master_transmit_receive(s_as5600_handle, &reg_addr, 1, data, 2, AS5600_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        // AS5600 registers are 12-bit: upper 4 bits are in MSB register, lower 8 bits in LSB register
        *out_val = (((uint16_t)(data[0] & 0x0F)) << 8) | (uint16_t)data[1];
    }
    return err;
}

// -----------------------------------------------------------------------------
// High-Level AS5600 Reading & Diagnostics
// -----------------------------------------------------------------------------
esp_err_t as5600_read_raw_angle(uint8_t motor_id, uint16_t *out_raw)
{
    return as5600_read_reg_u16(motor_id, AS5600_REG_RAW_ANGLE_MSB, out_raw);
}

esp_err_t as5600_read_diagnostics(uint8_t motor_id, as5600_diag_t *diag)
{
    if (diag == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t status_byte = 0;
    esp_err_t err = as5600_read_reg_u8(motor_id, AS5600_REG_STATUS, &status_byte);
    if (err != ESP_OK) return err;

    uint8_t agc_byte = 0;
    as5600_read_reg_u8(motor_id, AS5600_REG_AGC, &agc_byte);

    uint16_t mag_val = 0;
    as5600_read_reg_u16(motor_id, AS5600_REG_MAGNITUDE_MSB, &mag_val);

    diag->raw_status = status_byte;
    diag->magnet_detected = (status_byte & AS5600_STATUS_MD) != 0;
    diag->magnet_too_weak = (status_byte & AS5600_STATUS_ML) != 0;
    diag->magnet_too_strong = (status_byte & AS5600_STATUS_MH) != 0;
    diag->agc = agc_byte;
    diag->magnitude = mag_val;

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Multi-Motor Initialization
// -----------------------------------------------------------------------------
esp_err_t as5600_init_all(void)
{
    ESP_LOGI(TAG, "Initializing %d AS5600 Encoders via TCA9548A...", AS5600_NUM_ENCODERS);
    int detected_count = 0;

    for (uint8_t i = 0; i < AS5600_NUM_ENCODERS; i++) {
        encoders[i].motor_id = i;
        encoders[i].mux_channel = (int8_t)i;                  // Channel 0..3 on TCA9548A
        encoders[i].gear_ratio = AS5600_DEFAULT_GEAR_RATIO;   // 20:1 reduction
        encoders[i].invert_direction = true;                  // Gearbox inverts rotation
        encoders[i].connected = false;
        encoders[i].initialized_pos = false;
        encoders[i].raw_angle = 0;
        encoders[i].prev_raw_angle = 0;
        encoders[i].total_ticks = 0;
        encoders[i].zero_offset_ticks = 0;
        encoders[i].motor_angle_deg = 0.0f;
        encoders[i].motor_revs = 0.0f;
        encoders[i].output_angle_deg = 0.0f;
        encoders[i].output_angle_norm = 0.0f;
        encoders[i].output_revs = 0.0f;
        encoders[i].output_rpm = 0.0f;
        encoders[i].output_deg_s = 0.0f;
        encoders[i].last_sample_time_us = esp_timer_get_time();
        encoders[i].prev_ticks_for_vel = 0;
        memset(&encoders[i].diag, 0, sizeof(encoders[i].diag));

        // Attempt initial probe
        uint16_t test_raw = 0;
        esp_err_t err = as5600_read_raw_angle(i, &test_raw);
        if (err == ESP_OK) {
            encoders[i].connected = true;
            encoders[i].raw_angle = test_raw;
            encoders[i].prev_raw_angle = test_raw;
            encoders[i].initialized_pos = true;
            as5600_read_diagnostics(i, &encoders[i].diag);

            detected_count++;
            ESP_LOGI(TAG, "Encoder %u [TCA Ch %d]: ONLINE | Raw=%u | MD=%d | AGC=%u | Mag=%u",
                     i, i, test_raw,
                     encoders[i].diag.magnet_detected ? 1 : 0,
                     encoders[i].diag.agc,
                     encoders[i].diag.magnitude);
        } else {
            ESP_LOGW(TAG, "Encoder %u [TCA Ch %d]: NOT DETECTED (%s)",
                     i, i, esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "AS5600 discovery complete: %d / %d active encoders",
             detected_count, AS5600_NUM_ENCODERS);
    return (detected_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// -----------------------------------------------------------------------------
// Kinematics Update & Continuous Multi-Turn Unwrapping
// -----------------------------------------------------------------------------
esp_err_t as5600_update(uint8_t motor_id)
{
    if (motor_id >= AS5600_NUM_ENCODERS) {
        return ESP_ERR_INVALID_ARG;
    }
    as5600_encoder_t *enc = &encoders[motor_id];

    uint16_t raw_angle = 0;
    esp_err_t err = as5600_read_raw_angle(motor_id, &raw_angle);
    if (err != ESP_OK) {
        enc->connected = false;
        return err;
    }
    enc->connected = true;
    enc->raw_angle = raw_angle;

    // 1. Initial sample baseline
    if (!enc->initialized_pos) {
        enc->prev_raw_angle = raw_angle;
        enc->total_ticks = 0;
        enc->zero_offset_ticks = 0;
        enc->prev_ticks_for_vel = 0;
        enc->last_sample_time_us = esp_timer_get_time();
        enc->initialized_pos = true;
        return ESP_OK;
    }

    // 2. Unwrapped multi-turn tracking across 0 <-> 4095 boundary
    int32_t diff = (int32_t)raw_angle - (int32_t)enc->prev_raw_angle;
    if (diff > 2048) {
        // Wrapped backward across zero (e.g. 10 -> 4090)
        diff -= 4096;
    } else if (diff < -2048) {
        // Wrapped forward across zero (e.g. 4090 -> 10)
        diff += 4096;
    }

    enc->total_ticks += diff;
    enc->prev_raw_angle = raw_angle;

    // 3. Compute motor shaft position
    int64_t rel_ticks = enc->total_ticks - enc->zero_offset_ticks;
    enc->motor_revs = (float)rel_ticks / AS5600_COUNTS_PER_REV;
    enc->motor_angle_deg = enc->motor_revs * 360.0f;

    // 4. Compute 20:1 gearbox output shaft position
    // Direction inversion: gearbox reverses rotation
    float dir_sign = enc->invert_direction ? -1.0f : 1.0f;
    enc->output_revs = dir_sign * (enc->motor_revs / enc->gear_ratio);
    enc->output_angle_deg = enc->output_revs * 360.0f;

    // Normalized [0..360) degrees
    float norm_angle = fmodf(enc->output_angle_deg, 360.0f);
    if (norm_angle < 0.0f) {
        norm_angle += 360.0f;
    }
    enc->output_angle_norm = norm_angle;

    // 5. Velocity estimation (RPM & deg/s) with 30ms window
    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = now_us - enc->last_sample_time_us;
    if (dt_us >= 30000) { // 30 ms
        float dt_sec = (float)dt_us / 1000000.0f;
        int64_t delta_ticks = enc->total_ticks - enc->prev_ticks_for_vel;
        float d_motor_revs = (float)delta_ticks / AS5600_COUNTS_PER_REV;
        float d_output_revs = dir_sign * (d_motor_revs / enc->gear_ratio);

        float instant_rpm = (d_output_revs / dt_sec) * 60.0f;
        float instant_deg_s = (d_output_revs * 360.0f) / dt_sec;

        // Exponential smoothing filter for velocity readout
        enc->output_rpm = (0.7f * enc->output_rpm) + (0.3f * instant_rpm);
        enc->output_deg_s = (0.7f * enc->output_deg_s) + (0.3f * instant_deg_s);

        enc->prev_ticks_for_vel = enc->total_ticks;
        enc->last_sample_time_us = now_us;
    }

    return ESP_OK;
}

esp_err_t as5600_update_all(void)
{
    esp_err_t last_err = ESP_OK;
    for (uint8_t i = 0; i < AS5600_NUM_ENCODERS; i++) {
        esp_err_t err = as5600_update(i);
        if (err != ESP_OK) {
            last_err = err;
        }
    }
    return last_err;
}

// -----------------------------------------------------------------------------
// Calibration, Zeroing & Query APIs
// -----------------------------------------------------------------------------
void as5600_zero_output_position(uint8_t motor_id)
{
    if (motor_id < AS5600_NUM_ENCODERS) {
        encoders[motor_id].zero_offset_ticks = encoders[motor_id].total_ticks;
        as5600_update(motor_id);
        ESP_LOGI(TAG, "Encoder %u output position zeroed at current physical angle", motor_id);
    }
}

void as5600_zero_all(void)
{
    for (uint8_t i = 0; i < AS5600_NUM_ENCODERS; i++) {
        as5600_zero_output_position(i);
    }
}

float as5600_get_output_angle_deg(uint8_t motor_id)
{
    return (motor_id < AS5600_NUM_ENCODERS) ? encoders[motor_id].output_angle_deg : 0.0f;
}

float as5600_get_output_angle_norm(uint8_t motor_id)
{
    return (motor_id < AS5600_NUM_ENCODERS) ? encoders[motor_id].output_angle_norm : 0.0f;
}

float as5600_get_output_revs(uint8_t motor_id)
{
    return (motor_id < AS5600_NUM_ENCODERS) ? encoders[motor_id].output_revs : 0.0f;
}

float as5600_get_motor_revs(uint8_t motor_id)
{
    return (motor_id < AS5600_NUM_ENCODERS) ? encoders[motor_id].motor_revs : 0.0f;
}

float as5600_get_output_rpm(uint8_t motor_id)
{
    return (motor_id < AS5600_NUM_ENCODERS) ? encoders[motor_id].output_rpm : 0.0f;
}

// -----------------------------------------------------------------------------
// Formatted Telemetry Reporting
// -----------------------------------------------------------------------------
void as5600_print_telemetry(void)
{
    printf("\n+-----+-----+---------+----------+-----------+------------+------------+-----------+----------+-----+\n");
    printf("| MOT | MUX | STATUS  | RAW_ANG  | MTR REVS  | OUT DEG    | OUT REVS   | OUT RPM   | MAGNET   | AGC |\n");
    printf("+-----+-----+---------+----------+-----------+------------+------------+-----------+----------+-----+\n");

    for (uint8_t i = 0; i < AS5600_NUM_ENCODERS; i++) {
        as5600_encoder_t *enc = &encoders[i];
        if (!enc->connected) {
            printf("|  %u  | CH%d | OFFLINE |   ----   |   -----   |   ------   |   ------   |   -----   |  NO COMM | --- |\n",
                   i, enc->mux_channel);
            continue;
        }

        // Periodically refresh magnet health
        as5600_read_diagnostics(i, &enc->diag);

        const char *mag_str = "GOOD";
        if (!enc->diag.magnet_detected) {
            mag_str = "MISSING";
        } else if (enc->diag.magnet_too_weak) {
            mag_str = "TOO WEAK";
        } else if (enc->diag.magnet_too_strong) {
            mag_str = "TOO NEAR";
        }

        printf("|  %u  | CH%d |  ONLINE |  %4u    |  %7.2f  | %9.2f° |  %8.3f  |  %7.1f  | %-8s | %3u |\n",
               i,
               enc->mux_channel,
               enc->raw_angle,
               enc->motor_revs,
               enc->output_angle_deg,
               enc->output_revs,
               enc->output_rpm,
               mag_str,
               enc->diag.agc);
    }
    printf("+-----+-----+---------+----------+-----------+------------+------------+-----------+----------+-----+\n\n");
}

