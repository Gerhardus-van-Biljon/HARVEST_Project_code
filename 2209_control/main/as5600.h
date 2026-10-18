/**
 * @file as5600.h
 * @brief AS5600 12-bit Magnetic Rotary Encoder Driver for ESP32-S3 (ESP-IDF v6.0).
 * 
 * Features:
 * - 12-bit absolute angle sensing (4096 counts / revolution)
 * - TCA9548A 8-channel I2C multiplexer support for multi-motor installations
 * - 20:1 gearbox kinematics & continuous multi-turn unwrapping
 * - Output shaft cumulative angle, revolutions, and RPM velocity estimation
 * - Reversible gearbox direction configuration
 * - Magnet health diagnostics (Presence, Field Low/High, AGC, Magnitude)
 * - Closed-loop step-loss cross-verification hooks with TMC2209 stepper driver
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Hardware & I2C Defaults
// -----------------------------------------------------------------------------
#define AS5600_I2C_DEFAULT_SDA       GPIO_NUM_17
#define AS5600_I2C_DEFAULT_SCL       GPIO_NUM_18
#define AS5600_I2C_FREQ_HZ           400000          // 400 kHz Fast-mode
#define AS5600_I2C_TIMEOUT_MS        50

#define AS5600_I2C_ADDR              0x36            // Factory fixed 7-bit I2C address
#define TCA9548A_DEFAULT_ADDR        0x70            // Default address (A0=A1=A2=GND)
#define AS5600_NUM_ENCODERS          4               // 4 motors (Joints 0..3)

#define AS5600_COUNTS_PER_REV        4096.0f         // 12-bit resolution
#define AS5600_DEFAULT_GEAR_RATIO    20.0f           // 20:1 reduction gearbox

// -----------------------------------------------------------------------------
// AS5600 Register Map
// -----------------------------------------------------------------------------
#define AS5600_REG_ZMCO              0x00            // OTP burn count
#define AS5600_REG_ZPOS_MSB          0x01            // Zero position MSB (bits 11:8)
#define AS5600_REG_ZPOS_LSB          0x02            // Zero position LSB (bits 7:0)
#define AS5600_REG_MPOS_MSB          0x03            // Maximum position MSB (bits 11:8)
#define AS5600_REG_MPOS_LSB          0x04            // Maximum position LSB (bits 7:0)
#define AS5600_REG_MANG_MSB          0x05            // Maximum angle MSB (bits 11:8)
#define AS5600_REG_MANG_LSB          0x06            // Maximum angle LSB (bits 7:0)
#define AS5600_REG_CONF_MSB          0x07            // Configuration MSB
#define AS5600_REG_CONF_LSB          0x08            // Configuration LSB
#define AS5600_REG_STATUS            0x0B            // Status register (MD, ML, MH)
#define AS5600_REG_RAW_ANGLE_MSB     0x0C            // Raw unscaled angle MSB (bits 11:8)
#define AS5600_REG_RAW_ANGLE_LSB     0x0D            // Raw unscaled angle LSB (bits 7:0)
#define AS5600_REG_ANGLE_MSB         0x0E            // Processed angle MSB (bits 11:8)
#define AS5600_REG_ANGLE_LSB         0x0F            // Processed angle LSB (bits 7:0)
#define AS5600_REG_AGC               0x1A            // Automatic Gain Control (0..255)
#define AS5600_REG_MAGNITUDE_MSB     0x1B            // CORDIC magnitude MSB (bits 11:8)
#define AS5600_REG_MAGNITUDE_LSB     0x1C            // CORDIC magnitude LSB (bits 7:0)
#define AS5600_REG_BURN              0xFF            // Non-volatile burn command

// STATUS Register Bitmasks (Register 0x0B)
#define AS5600_STATUS_MH             0x08            // Bit 3: Magnet too strong (too close)
#define AS5600_STATUS_ML             0x10            // Bit 4: Magnet too weak (too far)
#define AS5600_STATUS_MD             0x20            // Bit 5: Magnet detected (valid field)

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------

/**
 * @brief Magnet alignment and sensor health diagnostics.
 */
typedef struct {
    bool     magnet_detected;        // True if MD bit is set (valid magnet present)
    bool     magnet_too_weak;        // True if ML bit is set (air gap too large)
    bool     magnet_too_strong;      // True if MH bit is set (air gap too close)
    uint8_t  agc;                    // AGC value (0..255 at 5V, 0..128 at 3.3V)
    uint16_t magnitude;              // CORDIC magnitude (0..4095)
    uint8_t  raw_status;             // Raw 0x0B register byte
} as5600_diag_t;

/**
 * @brief Encoder configuration and kinematic state for a single motor/joint.
 */
typedef struct {
    uint8_t       motor_id;          // Associated motor index (0..3)
    int8_t        mux_channel;       // TCA9548A channel (0..7, or -1 for direct bus)
    bool          connected;         // Device detected and responsive on I2C
    float         gear_ratio;        // Gear reduction ratio (e.g. 20.0f)
    bool          invert_direction;  // True if gearbox reverses rotation direction

    // Raw single-turn reading
    uint16_t      raw_angle;         // Last sampled raw angle (0..4095)
    uint16_t      prev_raw_angle;    // Previous raw angle for wrap detection
    bool          initialized_pos;   // True after first successful sample

    // Continuous multi-turn unwrapping
    int64_t       total_ticks;       // Cumulative unwrapped counts from startup
    int64_t       zero_offset_ticks; // Software zero reference point

    // Kinematic outputs
    float         motor_angle_deg;   // Motor shaft continuous angle in degrees
    float         motor_revs;        // Motor shaft continuous revolutions
    float         output_angle_deg;  // Gearbox output shaft continuous angle (degrees)
    float         output_angle_norm; // Gearbox output shaft angle normalized [0..360)
    float         output_revs;       // Gearbox output shaft continuous revolutions
    float         output_rpm;        // Estimated output shaft velocity in RPM
    float         output_deg_s;      // Estimated output shaft velocity in deg/s

    // Velocity estimation state
    int64_t       last_sample_time_us;
    int64_t       prev_ticks_for_vel;

    // Diagnostics
    as5600_diag_t diag;
} as5600_encoder_t;

// Global array of encoder instances
extern as5600_encoder_t encoders[AS5600_NUM_ENCODERS];

// -----------------------------------------------------------------------------
// Driver Initialization & Multiplexer API
// -----------------------------------------------------------------------------

/**
 * @brief Initialize the ESP32-S3 I2C master bus using modern driver/i2c_master.h.
 * 
 * @param sda_pin GPIO pin for I2C SDA (e.g. GPIO_NUM_17)
 * @param scl_pin GPIO pin for I2C SCL (e.g. GPIO_NUM_18)
 * @param clk_speed_hz Bus frequency in Hz (typically 400000)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t as5600_i2c_bus_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz);

/**
 * @brief Select an active channel on the TCA9548A multiplexer.
 * 
 * @param channel Channel index (0..7)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tca9548a_select_channel(uint8_t channel);

/**
 * @brief Disable all channels on the TCA9548A multiplexer (isolates I2C buses).
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t tca9548a_disable_all(void);

/**
 * @brief Initialize all 4 AS5600 encoders with TCA9548A multiplexer channels 0..3,
 *        20:1 gear ratio, and inverted direction.
 * 
 * @return esp_err_t ESP_OK if at least one encoder is detected
 */
esp_err_t as5600_init_all(void);

// -----------------------------------------------------------------------------
// Sampling & Kinematics Update API
// -----------------------------------------------------------------------------

/**
 * @brief Read raw 12-bit angle from the AS5600 for a specific motor.
 * 
 * @param motor_id Motor index (0..3)
 * @param out_raw Pointer to receive 12-bit angle (0..4095)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t as5600_read_raw_angle(uint8_t motor_id, uint16_t *out_raw);

/**
 * @brief Read diagnostics (magnet detected, AGC, magnitude) for a motor encoder.
 * 
 * @param motor_id Motor index (0..3)
 * @param diag Pointer to receive diagnostics structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t as5600_read_diagnostics(uint8_t motor_id, as5600_diag_t *diag);

/**
 * @brief Sample an encoder, update unwrapped multi-turn counts, and compute
 *        motor & gearbox output kinematics (angles, revolutions, RPM).
 * 
 * @param motor_id Motor index (0..3)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t as5600_update(uint8_t motor_id);

/**
 * @brief Update all 4 encoders sequentially through the TCA9548A multiplexer.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t as5600_update_all(void);

// -----------------------------------------------------------------------------
// Calibration, Zeroing & Query API
// -----------------------------------------------------------------------------

/**
 * @brief Set the current output shaft position as 0 degrees (software zero reference).
 * 
 * @param motor_id Motor index (0..3)
 */
void as5600_zero_output_position(uint8_t motor_id);

/**
 * @brief Zero output positions on all connected encoders.
 */
void as5600_zero_all(void);

/**
 * @brief Get the continuous output shaft angle in degrees for a given motor.
 * 
 * @param motor_id Motor index (0..3)
 * @return float Angle in degrees (taking 20:1 reduction & direction into account)
 */
float as5600_get_output_angle_deg(uint8_t motor_id);

/**
 * @brief Get normalized output shaft angle within [0..360) degrees.
 * 
 * @param motor_id Motor index (0..3)
 * @return float Normalized angle in degrees
 */
float as5600_get_output_angle_norm(uint8_t motor_id);

/**
 * @brief Get continuous gearbox output shaft revolutions.
 * 
 * @param motor_id Motor index (0..3)
 * @return float Output revolutions
 */
float as5600_get_output_revs(uint8_t motor_id);

/**
 * @brief Get continuous motor shaft revolutions.
 * 
 * @param motor_id Motor index (0..3)
 * @return float Motor revolutions
 */
float as5600_get_motor_revs(uint8_t motor_id);

/**
 * @brief Get estimated output shaft velocity in RPM.
 * 
 * @param motor_id Motor index (0..3)
 * @return float Velocity in RPM
 */
float as5600_get_output_rpm(uint8_t motor_id);

/**
 * @brief Print a clean, formatted ASCII telemetry table containing
 *        motor shaft, gearbox output, velocity, and magnet health.
 */
void as5600_print_telemetry(void);

#ifdef __cplusplus
}
#endif

