/*
 * as5600.h - notes & headers for our AS5600 12-bit magnetic rotary encoders
 *
 * Quick summary of how this works on our robot arm:
 * - 5 joints total (motors 0 through 4).
 * - Encoders are placed directly on the REAR motor shaft (not on the output shaft!).
 *   So 1 motor turn = exactly 4096 ticks.
 *   Our gearboxes (default 20:1) reduce that to the joint output.
 * - AS5600 chips have a hardcoded I2C address of 0x36. Since they all share
 *   the same address, we route them through a TCA9548A multiplexer (address 0x70)
 *   on channels 0 to 4.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C pins on our ESP32-S3 board and bus speed
#define AS5600_I2C_DEFAULT_SDA       GPIO_NUM_17
#define AS5600_I2C_DEFAULT_SCL       GPIO_NUM_18
#define AS5600_I2C_FREQ_HZ           400000          // 400 kHz fast mode
#define AS5600_I2C_TIMEOUT_MS        50

// addresses on the I2C bus
#define AS5600_I2C_ADDR              0x36            // fixed factory address for AS5600
#define TCA9548A_DEFAULT_ADDR        0x70            // default when A0=A1=A2=GND
#define AS5600_NUM_ENCODERS          5               // 5 joints/motors in our arm

// resolution & gearbox
#define AS5600_COUNTS_PER_REV        4096.0f         // 12-bit resolution on motor shaft
#define AS5600_DEFAULT_GEAR_RATIO    20.0f           // default 20:1 gearbox reduction

// registers we care about on the AS5600
#define AS5600_REG_ZMCO              0x00            // how many times zero was burned to OTP
#define AS5600_REG_ZPOS_MSB          0x01            // zero position high byte
#define AS5600_REG_ZPOS_LSB          0x02            // zero position low byte
#define AS5600_REG_MPOS_MSB          0x03            // max position high byte
#define AS5600_REG_MPOS_LSB          0x04            // max position low byte
#define AS5600_REG_MANG_MSB          0x05            // max angle high byte
#define AS5600_REG_MANG_LSB          0x06            // max angle low byte
#define AS5600_REG_CONF_MSB          0x07            // config high byte
#define AS5600_REG_CONF_LSB          0x08            // config low byte
#define AS5600_REG_STATUS            0x0B            // status (MD, ML, MH flags)
#define AS5600_REG_RAW_ANGLE_MSB     0x0C            // raw angle high 4 bits
#define AS5600_REG_RAW_ANGLE_LSB     0x0D            // raw angle low 8 bits
#define AS5600_REG_ANGLE_MSB         0x0E            // filtered angle high byte
#define AS5600_REG_ANGLE_LSB         0x0F            // filtered angle low byte
#define AS5600_REG_AGC               0x1A            // auto gain control (tells us field strength)
#define AS5600_REG_MAGNITUDE_MSB     0x1B            // cordic magnitude high byte
#define AS5600_REG_MAGNITUDE_LSB     0x1C            // cordic magnitude low byte
#define AS5600_REG_BURN              0xFF            // permanent OTP burn command

// status bit flags from register 0x0B (super helpful to check magnet alignment)
#define AS5600_STATUS_MH             0x08            // magnet is too close
#define AS5600_STATUS_ML             0x10            // magnet is too far away
#define AS5600_STATUS_MD             0x20            // magnet detected and within good range

// health check data for the magnet on a joint
typedef struct {
    bool     magnet_detected;        // MD bit: magnet is present and working
    bool     magnet_too_weak;        // ML bit: air gap is too big
    bool     magnet_too_strong;      // MH bit: air gap is too tight
    uint8_t  agc;                    // automatic gain control value (0..255)
    uint16_t magnitude;              // internal cordic magnitude reading
    uint8_t  raw_status;             // full byte from register 0x0B
} as5600_diag_t;

// holds all tracking state for a single joint's encoder
typedef struct {
    uint8_t       motor_id;          // which joint this belongs to (0..4)
    int8_t        mux_channel;       // channel on the TCA9548A (0..4)
    bool          connected;         // did it respond on I2C?
    float         gear_ratio;        // gearbox ratio (usually 20.0)
    bool          invert_direction;  // true if gearbox flips the spin direction

    // raw angle straight from the chip
    uint16_t      raw_angle;         // current reading (0..4095)
    uint16_t      prev_raw_angle;    // previous reading so we can catch rollover
    bool          initialized_pos;   // got our first baseline reading

    // unwrapped multi-turn tracking
    int64_t       total_ticks;       // continuous count from startup
    int64_t       zero_offset_ticks; // software zero point

    // calculated angles and speed
    float         motor_angle_deg;   // motor shaft continuous degrees
    float         motor_revs;        // motor shaft continuous turns
    float         output_angle_deg;  // gearbox output shaft degrees
    float         output_angle_norm; // normalized output angle [0..360)
    float         output_revs;       // gearbox output shaft turns
    float         output_rpm;        // estimated output speed in RPM
    float         output_deg_s;      // estimated output speed in deg/s

    // internal timer for velocity estimation
    int64_t       last_sample_time_us;
    int64_t       prev_ticks_for_vel;

    // magnet diagnostics
    as5600_diag_t diag;
} as5600_encoder_t;

// global state array for all 5 encoders
extern as5600_encoder_t encoders[AS5600_NUM_ENCODERS];

// setup the I2C bus using ESP-IDF v6 i2c_master driver
esp_err_t as5600_i2c_bus_init(gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz);

// switch the TCA9548A mux to talk to channel 0..7
esp_err_t tca9548a_select_channel(uint8_t channel);

// turn off all mux channels to leave the bus quiet
esp_err_t tca9548a_disable_all(void);

// loop through and initialize all 5 encoders on mux channels 0..4
esp_err_t as5600_init_all(void);

// grab the raw 12-bit angle (0..4095) for a specific joint
esp_err_t as5600_read_raw_angle(uint8_t motor_id, uint16_t *out_raw);

// read the magnet health flags (presence, AGC, field strength)
esp_err_t as5600_read_diagnostics(uint8_t motor_id, as5600_diag_t *diag);

// sample the encoder, handle zero-crossing roll, and update motor + output kinematics
esp_err_t as5600_update(uint8_t motor_id);

// update all 5 encoders in order through the mux
esp_err_t as5600_update_all(void);

// zero out current joint angle as our new software 0 position
void as5600_zero_output_position(uint8_t motor_id);

// zero out all 5 joints at once
void as5600_zero_all(void);

// change gear ratio / reverse direction for a joint if it's different from default
void as5600_set_gear_ratio(uint8_t motor_id, float ratio, bool invert);

// get continuous output shaft angle in degrees
float as5600_get_output_angle_deg(uint8_t motor_id);

// get continuous motor shaft angle in degrees
float as5600_get_motor_angle_deg(uint8_t motor_id);

// get output shaft angle normalized to [0..360) degrees
float as5600_get_output_angle_norm(uint8_t motor_id);

// continuous output turns
float as5600_get_output_revs(uint8_t motor_id);

// continuous motor turns
float as5600_get_motor_revs(uint8_t motor_id);

// estimated output speed in RPM
float as5600_get_output_rpm(uint8_t motor_id);

// dump a quick status table to the serial console
void as5600_print_telemetry(void);

#ifdef __cplusplus
}
#endif
