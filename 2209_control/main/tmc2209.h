/**
 * @file tmc2209.h
 * @brief TMC2209 Stepper Driver Register Definitions, Protocol, and Multi-Motor Control API.
 * 
 * Supports up to 4 TMC2209 drivers on a shared half-duplex UART bus (addresses 0..3).
 * Provides full control over motion (STEP/DIR & VACTUAL), StallGuard4, CoolStep,
 * current scaling, microstepping, SpreadCycle/StealthChop, diagnostics, and robotics hooks.
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
// Hardware & Protocol Configuration
// -----------------------------------------------------------------------------
#define TMC_NUM_MOTORS            4       // Addresses 0, 1, 2, 3

// TMC2209 UART Register Addresses
#define REG_GCONF                 0x00    // Global configuration
#define REG_GSTAT                 0x01    // Global status flags (reset, drv_err, uv_cp)
#define REG_IFCNT                 0x02    // Interface transmission counter
#define REG_SLAVECONF             0x03    // Slave configuration (SENDDELAY)
#define REG_IOIN                  0x06    // Input pin states (ENN, STEP, DIR, MS1, MS2, VERSION)
#define REG_IHOLD_IRUN            0x10    // Driver current control (IHOLD, IRUN, IHOLDDELAY)
#define REG_TPOWERDOWN            0x11    // Standstill power-down delay
#define REG_TSTEP                 0x12    // Measured time between microsteps (velocity indicator)
#define REG_TPWMTHRS              0x13    // Upper velocity threshold for StealthChop
#define REG_TCOOLTHRS             0x14    // Lower velocity threshold for CoolStep & StallGuard
#define REG_VACTUAL               0x22    // Direct UART velocity control (signed 24-bit)
#define REG_SGTHRS                0x40    // StallGuard4 sensitivity threshold (0..255)
#define REG_SG_RESULT             0x41    // StallGuard4 result value (0..510 load indicator)
#define REG_COOLCONF              0x42    // CoolStep smart energy configuration
#define REG_CHOPCONF              0x6C    // Chopper configuration & microstep resolution (MRES)
#define REG_DRV_STATUS            0x6F    // Driver status & diagnostic flags
#define REG_PWMCONF               0x70    // StealthChop PWM configuration & freewheeling mode
#define REG_PWM_SCALE             0x71    // Actual PWM scale
#define REG_PWM_AUTO              0x72    // Automatically determined PWM values

// Microstep Resolutions (MRES in CHOPCONF)
#define TMC_MRES_256              0       // Native 256 microsteps
#define TMC_MRES_128              1       // 128 microsteps
#define TMC_MRES_64               2       // 64 microsteps
#define TMC_MRES_32               3       // 32 microsteps
#define TMC_MRES_16               4       // 16 microsteps
#define TMC_MRES_8                5       // 8 microsteps
#define TMC_MRES_4                6       // 4 microsteps
#define TMC_MRES_2                7       // 2 microsteps (halfstep)
#define TMC_MRES_1                8       // Fullstep

// Standstill Freewheeling Modes (PWMCONF bits 14..15)
#define TMC_FREEWHEEL_NORMAL      0       // Normal operation (standstill current IHOLD)
#define TMC_FREEWHEEL_COIL_OFF    1       // Passive free-wheeling (coils unpowered)
#define TMC_FREEWHEEL_SHORT_GND   2       // Coils shorted to GND (passive inductive braking)
#define TMC_FREEWHEEL_SHORT_VS    3       // Coils shorted to VS

// Motion Modes
typedef enum {
    MOTION_MODE_STEP_DIR = 0,             // Pulsed hardware STEP / DIR control
    MOTION_MODE_UART_VACTUAL = 1          // Direct UART internal velocity generator
} tmc_motion_mode_t;

// -----------------------------------------------------------------------------
// Driver Diagnostics Telemetry Structure
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t  addr;                        // Driver address (0..3)
    bool     otpw;                        // Overtemperature pre-warning
    bool     ot;                          // Overtemperature shutdown
    bool     s2ga;                        // Short-to-ground on Phase A
    bool     s2gb;                        // Short-to-ground on Phase B
    bool     ola;                         // Open load on Phase A
    bool     olb;                         // Open load on Phase B
    uint8_t  cs_actual;                   // Actual current scale (0..31)
    bool     stealthchop;                 // StealthChop active indicator
    bool     standstill;                  // Standstill active indicator
    uint16_t stallguard_result;           // StallGuard4 load value (0..510)
    uint32_t raw_status;                  // Full 32-bit DRV_STATUS register
} tmc_drv_status_t;

// -----------------------------------------------------------------------------
// Motor Configuration & State Structure
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t           addr;               // Driver address on UART bus (0..3)
    bool              present;            // Driver detected and responsive on UART
    bool              configured;         // Initial register configuration completed
    bool              enabled;            // Driver enabled (EN pin + software TOFF)
    
    // Hardware GPIO pins
    gpio_num_t        step_pin;           // Hardware STEP pulse GPIO
    gpio_num_t        dir_pin;            // Hardware DIR direction GPIO
    gpio_num_t        en_pin;             // Hardware EN enable GPIO (can be shared)
    bool              dir_level;          // Hardware DIR pin level (0 or 1)
    
    // Motion control state
    tmc_motion_mode_t motion_mode;        // Hardware STEP/DIR or UART VACTUAL
    int               target_speed_hz;    // Commanded step frequency in Hz
    int               current_speed_hz;   // Current active step frequency in Hz
    int32_t           step_accum_us;      // Microsecond accumulator for multi-motor timer
    int32_t           vactual_velocity;   // Velocity commanded in VACTUAL mode
    
    // StallGuard4 & Load telemetry
    uint8_t           sg_threshold;       // StallGuard threshold (SGTHRS: 0..255)
    uint16_t          last_sg_result;     // Last measured SG_RESULT (0..510)
    bool              is_stalled;         // Stall detected flag
    
    // Current & Chopper settings
    uint8_t           irun;               // Run current (0..31)
    uint8_t           ihold;              // Standstill current (0..31)
    uint8_t           iholddelay;         // Delay to standstill current (0..15)
    uint16_t          microsteps;         // Microstep division (1, 2, 4, 8, 16, 32, 64, 128, 256)
    bool              reverse_dir;        // Software direction inversion via UART (shaft bit)
    bool              spreadcycle_en;     // SpreadCycle enabled (true) or StealthChop (false)
    
    // Commanded position & step tracking
    int64_t           total_commanded_steps; // Cumulative emitted microstep pulses
    float             commanded_motor_deg;   // Theoretical motor angle from steps
    float             commanded_output_deg;  // Theoretical 20:1 output angle from steps
    float             tracking_error_deg;    // Delta between commanded and AS5600 angle

    // Shadow registers for safe read-modify-write operations
    uint32_t          shadow_gconf;
    uint32_t          shadow_chopconf;
    uint32_t          shadow_pwmconf;
} tmc_motor_t;

// Global array containing configuration & state for all 4 motors
extern tmc_motor_t motors[TMC_NUM_MOTORS];

// -----------------------------------------------------------------------------
// Core UART Protocol & Register Access
// -----------------------------------------------------------------------------
void tmc_calc_crc(uint8_t *datagram, uint8_t datagram_length);
void tmc_write_reg(uint8_t addr, uint8_t reg, uint32_t value);
bool tmc_read_reg_ex(uint8_t addr, uint8_t reg, uint32_t *out, bool verbose);
bool tmc_read_reg(uint8_t addr, uint8_t reg, uint32_t *out);

// -----------------------------------------------------------------------------
// Multi-Motor Initialization & Bus Discovery
// -----------------------------------------------------------------------------
void tmc_init_motor_structures(void);
void tmc_init_hardware_pins(void);
bool tmc_probe_motor(uint8_t addr, bool verbose);
bool tmc_configure_motor(uint8_t addr);
void tmc_scan_and_configure_all(void);

// -----------------------------------------------------------------------------
// StallGuard4 & Sensorless Homing API
// -----------------------------------------------------------------------------
bool tmc_stallguard_set_threshold(uint8_t addr, uint8_t threshold);
bool tmc_stallguard_get_result(uint8_t addr, uint16_t *out_result);
bool tmc_stallguard_enable(uint8_t addr, uint8_t threshold, uint32_t tcoolthrs);
bool tmc_stallguard_check_stall(uint8_t addr, bool *is_stalled);
void tmc_clear_stall(uint8_t addr);
bool tmc_sensorless_homing(uint8_t addr, int32_t homing_velocity, uint8_t sg_threshold);

// -----------------------------------------------------------------------------
// CoolStep (Smart Energy & Automatic Current Adjustment)
// -----------------------------------------------------------------------------
bool tmc_coolstep_enable(uint8_t addr, uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin);
bool tmc_coolstep_disable(uint8_t addr);

// -----------------------------------------------------------------------------
// Current, Microstepping & Chopper Tuning API
// -----------------------------------------------------------------------------
bool tmc_set_current(uint8_t addr, uint8_t irun, uint8_t ihold, uint8_t iholddelay);
bool tmc_set_microsteps(uint8_t addr, uint16_t microsteps, bool interpolate);
bool tmc_set_spreadcycle(uint8_t addr, bool enable_spreadcycle);
bool tmc_set_stealthchop_pwm_threshold(uint8_t addr, uint32_t tpwmthrs);

// -----------------------------------------------------------------------------
// Motion & Velocity Control (UART VACTUAL and STEP/DIR)
// -----------------------------------------------------------------------------
bool tmc_set_velocity_vactual(uint8_t addr, int32_t velocity);
bool tmc_stop_vactual(uint8_t addr);
void tmc_set_motor_speed_hz(uint8_t addr, int speed_hz);
void tmc_set_motor_direction(uint8_t addr, bool forward);
void tmc_set_motor_enable(uint8_t addr, bool enable);
bool tmc_set_shaft_direction_uart(uint8_t addr, bool reverse);
bool tmc_set_software_enable(uint8_t addr, bool enable);
bool tmc_set_freewheel_mode(uint8_t addr, uint8_t mode);
bool tmc_set_standby_delay(uint8_t addr, uint8_t delay);

// Multi-motor simultaneous controls
void tmc_run_all_motors(int speed_hz);
void tmc_stop_all_motors(void);
void tmc_emergency_stop_all(void);

// -----------------------------------------------------------------------------
// Telemetry & Diagnostics API
// -----------------------------------------------------------------------------
bool tmc_get_drv_status(uint8_t addr, tmc_drv_status_t *status);
bool tmc_get_ioin(uint8_t addr, uint32_t *ioin_val);
bool tmc_clear_gstat(uint8_t addr);
void tmc_print_motor_status(uint8_t addr);
void tmc_print_all_motors_status(void);
bool tmc_get_position_tracking_error(uint8_t addr, float *out_error_deg);
void tmc_reset_position(uint8_t addr);

// -----------------------------------------------------------------------------
// Kinematics & Trajectory Hooks (Dummy / Extensible Callbacks)
// -----------------------------------------------------------------------------
void robot_arm_joint_move_dummy(uint8_t joint_id, float target_angle_deg, float speed_deg_s);
void robot_arm_stream_trajectory_dummy(const float joint_angles[4], const float joint_speeds[4]);
void robot_arm_stallguard_event_callback(uint8_t joint_id, uint16_t sg_result);
void robot_arm_sync_home_all_dummy(void);

#ifdef __cplusplus
}
#endif

