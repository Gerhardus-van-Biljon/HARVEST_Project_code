/*
 * tmc2209.h - register map, configuration, and controls for our 5 TMC2209 drivers
 *
 * Project notes:
 * - 5 joints total on the arm.
 * - Hardware UART addressing: TMC2209 MS1/MS2 pins only give 4 addresses (0..3).
 *   So we split across two UART ports on the ESP32-S3:
 *     * Bus 0 (UART1): Motors 0..3 (addresses 0, 1, 2, 3)
 *     * Bus 1 (UART2): Motor 4 (address 0)
 * - Motor types & currents:
 *     * Motor 0: big 60mm 2A NEMA 17 for the base (run at IRUN=25, ~1.44A RMS)
 *     * Motors 1..4: 17HS3401S NEMA 17s (run at IRUN=16, ~0.94A RMS to keep them cool)
 * - StallGuard4 note:
 *     * Operates in StealthChop mode (spreadcycle_en = false).
 *     * At standstill back-EMF is 0, so SG_RESULT reads 0 naturally.
 *       Never flag a stall when the motor is stopped or starting up!
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 5 joints in total
#define TMC_NUM_MOTORS            5

// Dual UART bus wiring on ESP32-S3
// Bus 0 handles Motors 0..3 (Addresses 0..3 via MS1/MS2)
#define TMC_UART_BUS0             UART_NUM_1
#define TMC_UART_BUS0_TX_PIN      GPIO_NUM_2      // connects to PDN_UART through 1k resistor
#define TMC_UART_BUS0_RX_PIN      GPIO_NUM_1      // connects directly to PDN_UART line

// Bus 1 handles Motor 4 (Address 0)
#define TMC_UART_BUS1             UART_NUM_2
#define TMC_UART_BUS1_TX_PIN      GPIO_NUM_12     // connects to Motor 4 PDN_UART through 1k resistor
#define TMC_UART_BUS1_RX_PIN      GPIO_NUM_11     // connects directly to Motor 4 PDN_UART line

// Step/Dir pins for Motor 4 (Motors 0..3 are defined in tmc2209.c)
#define MOTOR4_STEP_PIN           GPIO_NUM_48
#define MOTOR4_DIR_PIN            GPIO_NUM_47
#define MOTOR4_EN_PIN             GPIO_NUM_15     // shared enable pin across all 5 drivers

// TMC2209 UART registers we talk to
#define REG_GCONF                 0x00    // global config (StealthChop, direction, etc)
#define REG_GSTAT                 0x01    // reset & error flags
#define REG_IFCNT                 0x02    // transmission counter (handy to verify writes)
#define REG_SLAVECONF             0x03    // reply delay config
#define REG_IOIN                  0x06    // reads live pin states (step, dir, en, etc)
#define REG_IHOLD_IRUN            0x10    // run current, hold current, and hold delay
#define REG_TPOWERDOWN            0x11    // delay before dropping down to hold current
#define REG_TSTEP                 0x12    // measured time between microsteps (velocity)
#define REG_TPWMTHRS              0x13    // max speed threshold for StealthChop
#define REG_TCOOLTHRS             0x14    // min speed threshold for CoolStep and StallGuard
#define REG_VACTUAL               0x22    // direct UART velocity mode (driver steps itself)
#define REG_SGTHRS                0x40    // StallGuard4 sensitivity threshold (0..255)
#define REG_SG_RESULT             0x41    // StallGuard4 live mechanical load reading (0..510)
#define REG_COOLCONF              0x42    // CoolStep current saving settings
#define REG_CHOPCONF              0x6C    // chopper config & microstep resolution (MRES)
#define REG_DRV_STATUS            0x6F    // driver status flags (standstill, overtemp, shorts)
#define REG_PWMCONF               0x70    // StealthChop tuning and freewheel settings
#define REG_PWM_SCALE             0x71    // live PWM scaling
#define REG_PWM_AUTO              0x72    // auto-tuned PWM values

// microstep resolution settings in CHOPCONF
#define TMC_MRES_256              0       // 256 microsteps
#define TMC_MRES_128              1       // 128 microsteps
#define TMC_MRES_64               2       // 64 microsteps
#define TMC_MRES_32               3       // 32 microsteps
#define TMC_MRES_16               4       // 16 microsteps
#define TMC_MRES_8                5       // 8 microsteps
#define TMC_MRES_4                6       // 4 microsteps
#define TMC_MRES_2                7       // half-step
#define TMC_MRES_1                8       // full-step

// what to do with motor coils when stopped (PWMCONF bits 14..15)
#define TMC_FREEWHEEL_NORMAL      0       // normal hold current (IHOLD)
#define TMC_FREEWHEEL_COIL_OFF    1       // coils totally unpowered (freewheeling)
#define TMC_FREEWHEEL_SHORT_GND   2       // coils shorted to GND (passive brake)
#define TMC_FREEWHEEL_SHORT_VS    3       // coils shorted to VS

// motion control modes
typedef enum {
    MOTION_MODE_STEP_DIR = 0,             // hardware STEP/DIR pulses from ESP32 timer
    MOTION_MODE_UART_VACTUAL = 1          // autonomous velocity mode inside driver chip
} tmc_motion_mode_t;

// diagnostic telemetry decoded from DRV_STATUS
typedef struct {
    uint8_t  addr;                        // driver address
    bool     otpw;                        // overtemperature warning
    bool     ot;                          // overtemperature shutdown triggered
    bool     s2ga;                        // phase A shorted to ground
    bool     s2gb;                        // phase B shorted to ground
    bool     ola;                         // phase A open load (disconnected wire?)
    bool     olb;                         // phase B open load
    uint8_t  cs_actual;                   // active current scale (0..31)
    bool     stealthchop;                 // StealthChop currently active
    bool     standstill;                  // driver detects motor is stopped
    uint16_t stallguard_result;           // StallGuard4 load value (0..510)
    uint32_t raw_status;                  // raw 32-bit DRV_STATUS register
} tmc_drv_status_t;

// full state and config for a single motor
typedef struct {
    uint8_t           motor_id;           // global joint index (0..4)
    uint8_t           bus_id;             // UART bus (0 or 1)
    int               uart_port;          // UART_NUM_1 or UART_NUM_2
    uint8_t           uart_addr;          // address on that specific UART bus (0..3)
    uint8_t           addr;               // legacy alias
    bool              present;            // did we find this driver on boot?
    bool              configured;         // registers initialized?
    bool              enabled;            // motor enabled
    
    // GPIO pins
    gpio_num_t        step_pin;           // STEP pulse pin
    gpio_num_t        dir_pin;            // DIR direction pin
    gpio_num_t        en_pin;             // EN enable pin (active LOW)
    bool              dir_level;          // DIR pin level (0 or 1)
    
    // motion state
    tmc_motion_mode_t motion_mode;        // STEP/DIR or UART VACTUAL
    int               target_speed_hz;    // commanded step rate
    int               current_speed_hz;   // active step rate (ramped)
    int32_t           step_accum_us;      // accumulator for timer pulses
    int32_t           vactual_velocity;   // velocity if running VACTUAL
    
    // StallGuard4 settings
    uint8_t           sg_threshold;       // stall threshold (0..255)
    uint16_t          last_sg_result;     // last measured SG_RESULT
    bool              is_stalled;         // stall flag
    
    // current & chopper settings
    uint8_t           irun;               // run current (0..31)
    uint8_t           ihold;              // hold current (0..31)
    uint8_t           iholddelay;         // power-down delay
    uint16_t          microsteps;         // microstep divider (e.g. 16)
    bool              reverse_dir;        // flip spin direction via UART
    bool              spreadcycle_en;     // false = StealthChop, true = SpreadCycle
    
    // step tracking & position comparison with AS5600
    int64_t           total_commanded_steps; // total pulses sent
    float             commanded_motor_deg;   // theoretical motor degrees from steps
    float             commanded_output_deg;  // theoretical output degrees after gearbox
    float             tracking_error_deg;    // difference between commanded and actual output
    float             tracking_error_motor_deg; // difference directly on the motor shaft

    // shadow copies of registers so we can do bitwise updates safely
    uint32_t          shadow_gconf;
    uint32_t          shadow_chopconf;
    uint32_t          shadow_pwmconf;
} tmc_motor_t;

// global array of all 5 motors
extern tmc_motor_t motors[TMC_NUM_MOTORS];

// low-level UART protocol functions
void tmc_calc_crc(uint8_t *datagram, uint8_t datagram_length);
esp_err_t tmc_uart_bus_init(void);
void tmc_write_reg_bus(int uart_port, uint8_t bus_addr, uint8_t reg, uint32_t value);
bool tmc_read_reg_bus(int uart_port, uint8_t bus_addr, uint8_t reg, uint32_t *out, bool verbose);
void tmc_write_reg(uint8_t addr, uint8_t reg, uint32_t value);
bool tmc_read_reg_ex(uint8_t addr, uint8_t reg, uint32_t *out, bool verbose);
bool tmc_read_reg(uint8_t addr, uint8_t reg, uint32_t *out);

// motor setup & bus discovery
void tmc_init_motor_structures(void);
void tmc_init_hardware_pins(void);
bool tmc_probe_motor(uint8_t addr, bool verbose);
bool tmc_configure_motor(uint8_t addr);
void tmc_scan_and_configure_all(void);

// StallGuard4 & sensorless homing
bool tmc_stallguard_set_threshold(uint8_t addr, uint8_t threshold);
bool tmc_stallguard_get_result(uint8_t addr, uint16_t *out_result);
bool tmc_stallguard_enable(uint8_t addr, uint8_t threshold, uint32_t tcoolthrs);
bool tmc_stallguard_check_stall(uint8_t addr, bool *is_stalled);
bool tmc_sensorless_homing(uint8_t addr, int32_t homing_velocity, uint8_t sg_threshold);

// CoolStep auto current reduction
bool tmc_coolstep_enable(uint8_t addr, uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin);
bool tmc_coolstep_disable(uint8_t addr);

// current and microstep tuning
bool tmc_set_current(uint8_t addr, uint8_t irun, uint8_t ihold, uint8_t iholddelay);
bool tmc_set_microsteps(uint8_t addr, uint16_t microsteps, bool interpolate);
bool tmc_set_spreadcycle(uint8_t addr, bool enable_spreadcycle);
bool tmc_set_stealthchop_pwm_threshold(uint8_t addr, uint32_t tpwmthrs);

// motion commands
bool tmc_set_velocity_vactual(uint8_t addr, int32_t velocity);
bool tmc_stop_vactual(uint8_t addr);
void tmc_set_motor_speed_hz(uint8_t addr, int speed_hz);
void tmc_set_motor_direction(uint8_t addr, bool forward);
void tmc_set_motor_enable(uint8_t addr, bool enable);
bool tmc_set_shaft_direction_uart(uint8_t addr, bool reverse);
bool tmc_set_software_enable(uint8_t addr, bool enable);
bool tmc_set_freewheel_mode(uint8_t addr, uint8_t mode);
bool tmc_set_standby_delay(uint8_t addr, uint8_t delay);

// multi-motor controls
void tmc_run_all_motors(int speed_hz);
void tmc_stop_all_motors(void);
void tmc_emergency_stop_all(void);

// telemetry & diagnostics
bool tmc_get_drv_status(uint8_t addr, tmc_drv_status_t *status);
bool tmc_get_ioin(uint8_t addr, uint32_t *ioin_val);
bool tmc_clear_gstat(uint8_t addr);
void tmc_print_motor_status(uint8_t addr);
void tmc_print_all_motors_status(void);
bool tmc_get_position_tracking_error(uint8_t addr, float *out_error_deg);
void tmc_reset_position(uint8_t addr);

// robotics trajectory & safety hooks
void robot_arm_joint_move_dummy(uint8_t joint_id, float target_angle_deg, float speed_deg_s);
void robot_arm_stream_trajectory_dummy(const float joint_angles[5], const float joint_speeds[5]);
void robot_arm_stallguard_event_callback(uint8_t joint_id, uint16_t sg_result);
void robot_arm_sync_home_all_dummy(void);

#ifdef __cplusplus
}
#endif
