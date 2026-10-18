/**
 * @file tmc2209.c
 * @brief TMC2209 Multi-Driver Implementation for ESP32-S3.
 */

#include "tmc2209.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/uart.h"

static const char *TAG = "TMC2209";

// UART Port & Pin definitions
#define UART_PORT         UART_NUM_1
#define UART_TX_PIN       GPIO_NUM_10     // -> driver RX
#define UART_RX_PIN       GPIO_NUM_11     // <- driver TX

// Default pinout for 4 motors (ESP32-S3 GPIOs)
// Motor 0 (Address 0: MS1=GND, MS2=GND)
#define MOTOR0_STEP_PIN   GPIO_NUM_9
#define MOTOR0_DIR_PIN    GPIO_NUM_2
#define MOTOR0_EN_PIN     GPIO_NUM_4

// Motor 1 (Address 1: MS1=VIO, MS2=GND)
#define MOTOR1_STEP_PIN   GPIO_NUM_5
#define MOTOR1_DIR_PIN    GPIO_NUM_6
#define MOTOR1_EN_PIN     GPIO_NUM_4

// Motor 2 (Address 2: MS1=GND, MS2=VIO)
#define MOTOR2_STEP_PIN   GPIO_NUM_7
#define MOTOR2_DIR_PIN    GPIO_NUM_8
#define MOTOR2_EN_PIN     GPIO_NUM_4

// Motor 3 (Address 3: MS1=VIO, MS2=VIO)
#define MOTOR3_STEP_PIN   GPIO_NUM_12
#define MOTOR3_DIR_PIN    GPIO_NUM_13
#define MOTOR3_EN_PIN     GPIO_NUM_4

// Global state array for all 4 motors
tmc_motor_t motors[TMC_NUM_MOTORS];

// -----------------------------------------------------------------------------
// CRC8 Calculation (Trinamic Polynomial x^8 + x^2 + x^1 + 1, seed 0x07)
// -----------------------------------------------------------------------------
void tmc_calc_crc(uint8_t *datagram, uint8_t datagram_length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (datagram_length - 1); i++) {
        uint8_t currentByte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            currentByte >>= 1;
        }
    }
    datagram[datagram_length - 1] = crc;
}

// -----------------------------------------------------------------------------
// Low-Level Register Write (8-byte frame)
// -----------------------------------------------------------------------------
void tmc_write_reg(uint8_t addr, uint8_t reg, uint32_t value)
{
    uint8_t datagram[8] = {0};

    datagram[0] = 0x05;                         // Sync byte
    datagram[1] = addr;                         // Slave address (0..3)
    datagram[2] = (uint8_t)(reg | 0x80u);       // Register address with write bit (MSB=1)
    datagram[3] = (uint8_t)((value >> 24) & 0xFFu);
    datagram[4] = (uint8_t)((value >> 16) & 0xFFu);
    datagram[5] = (uint8_t)((value >> 8) & 0xFFu);
    datagram[6] = (uint8_t)(value & 0xFFu);

    tmc_calc_crc(datagram, sizeof(datagram));

    uart_write_bytes(UART_PORT, (const char *)datagram, sizeof(datagram));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));
    uart_flush_input(UART_PORT);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// -----------------------------------------------------------------------------
// Low-Level Register Read (4-byte request, 8-byte reply)
// -----------------------------------------------------------------------------
bool tmc_read_reg_ex(uint8_t addr, uint8_t reg, uint32_t *out, bool verbose)
{
    uint8_t req[4] = {0x05, addr, reg, 0};
    tmc_calc_crc(req, sizeof(req));

    uint8_t buf[16];
    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, (const char *)req, sizeof(req));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));

    int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));

    if (verbose) {
        printf("[UART] Probing Addr %u | TX: %02X %02X %02X %02X | RX (%d bytes):",
               addr, req[0], req[1], req[2], req[3], len);
        for (int i = 0; i < len; i++) printf(" %02X", buf[i]);
    }

    for (int i = 0; i + 8 <= len; i++) {
        // Trinamic response sync is 0x05, master reply address is 0xFF, reg must match
        if (buf[i] == 0x05 && buf[i + 1] == 0xFF && buf[i + 2] == reg) {
            uint8_t crc_buf[8];
            memcpy(crc_buf, &buf[i], 8);
            tmc_calc_crc(crc_buf, 8);
            if (crc_buf[7] != buf[i + 7]) {
                if (verbose) printf(" | Bad CRC!\n");
                return false;
            }
            if (out != NULL) {
                *out = ((uint32_t)buf[i + 3] << 24) |
                       ((uint32_t)buf[i + 4] << 16) |
                       ((uint32_t)buf[i + 5] << 8)  |
                        (uint32_t)buf[i + 6];
            }
            if (verbose) printf(" | VALID reply from Addr %u! Val=0x%08lX\n", addr, (unsigned long)(out ? *out : 0));
            return true;
        }
    }

    if (verbose) printf(" | No reply\n");
    return false;
}

bool tmc_read_reg(uint8_t addr, uint8_t reg, uint32_t *out)
{
    return tmc_read_reg_ex(addr, reg, out, false);
}

// -----------------------------------------------------------------------------
// Initialization & Pin Setup
// -----------------------------------------------------------------------------
void tmc_init_motor_structures(void)
{
    // Motor 0 (Address 0: MS1=GND, MS2=GND)
    motors[0].addr = 0;
    motors[0].step_pin = MOTOR0_STEP_PIN;
    motors[0].dir_pin  = MOTOR0_DIR_PIN;
    motors[0].en_pin   = MOTOR0_EN_PIN;

    // Motor 1 (Address 1: MS1=VIO, MS2=GND)
    motors[1].addr = 1;
    motors[1].step_pin = MOTOR1_STEP_PIN;
    motors[1].dir_pin  = MOTOR1_DIR_PIN;
    motors[1].en_pin   = MOTOR1_EN_PIN;

    // Motor 2 (Address 2: MS1=GND, MS2=VIO)
    motors[2].addr = 2;
    motors[2].step_pin = MOTOR2_STEP_PIN;
    motors[2].dir_pin  = MOTOR2_DIR_PIN;
    motors[2].en_pin   = MOTOR2_EN_PIN;

    // Motor 3 (Address 3: MS1=VIO, MS2=VIO)
    motors[3].addr = 3;
    motors[3].step_pin = MOTOR3_STEP_PIN;
    motors[3].dir_pin  = MOTOR3_DIR_PIN;
    motors[3].en_pin   = MOTOR3_EN_PIN;

    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].present = false;
        motors[i].configured = false;
        motors[i].enabled = true;
        motors[i].dir_level = 1;
        motors[i].motion_mode = MOTION_MODE_STEP_DIR;
        motors[i].target_speed_hz = 0;
        motors[i].current_speed_hz = 0;
        motors[i].step_accum_us = 0;
        motors[i].vactual_velocity = 0;
        motors[i].sg_threshold = 64;           // Default StallGuard threshold
        motors[i].last_sg_result = 0;
        motors[i].is_stalled = false;
        motors[i].irun = 31;                   // Full current
        motors[i].ihold = 16;                  // 50% hold current
        motors[i].iholddelay = 1;
        motors[i].microsteps = 4;              // 4 microsteps
        motors[i].reverse_dir = false;
        motors[i].spreadcycle_en = true;       // SpreadCycle mode enabled

        motors[i].total_commanded_steps = 0;
        motors[i].commanded_motor_deg   = 0.0f;
        motors[i].commanded_output_deg  = 0.0f;
        motors[i].tracking_error_deg    = 0.0f;

        // Initial shadow register values
        motors[i].shadow_gconf    = 0x000000C4u;  // SpreadCycle, pdn_disable, mstep_reg_select
        motors[i].shadow_chopconf = 0x16000054u;  // TOFF=4, HSTRT=5, MRES=6 (4 mstep), intpol=1
        motors[i].shadow_pwmconf  = 0xC10D0024u;  // StealthChop autoscale on
    }
}

void tmc_init_hardware_pins(void)
{
    uint64_t pin_mask = 0;
    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        pin_mask |= (1ULL << motors[i].step_pin);
        pin_mask |= (1ULL << motors[i].dir_pin);
        pin_mask |= (1ULL << motors[i].en_pin);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Initialize initial pin states
    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        gpio_set_level(motors[i].en_pin, 0);     // Active LOW = enabled
        gpio_set_level(motors[i].dir_pin, motors[i].dir_level);
        gpio_set_level(motors[i].step_pin, 0);
    }
}

// -----------------------------------------------------------------------------
// Motor Bus Probing & Configuration
// -----------------------------------------------------------------------------
bool tmc_probe_motor(uint8_t addr, bool verbose)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    uint32_t ifcnt;
    bool ok = tmc_read_reg_ex(addr, REG_IFCNT, &ifcnt, verbose);
    if (ok) {
        motors[addr].present = true;
    }
    return ok;
}

bool tmc_configure_motor(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;

    uint32_t ifcnt_before = 0, ifcnt_after = 0;
    bool have_before = tmc_read_reg(addr, REG_IFCNT, &ifcnt_before);

    // 1. GCONF: bit2 SpreadCycle, bit6 pdn_disable, bit7 mstep_reg_select
    motors[addr].shadow_gconf = 0x000000C4u;
    if (motors[addr].reverse_dir) {
        motors[addr].shadow_gconf |= (1u << 3);  // shaft bit
    }
    if (!motors[addr].spreadcycle_en) {
        motors[addr].shadow_gconf &= ~(1u << 2); // StealthChop
    }
    tmc_write_reg(addr, REG_GCONF, motors[addr].shadow_gconf);

    // 2. IHOLD_IRUN: IHOLD[4:0], IRUN[12:8], IHOLDDELAY[19:16]
    uint32_t ihold_irun = ((uint32_t)motors[addr].iholddelay << 16) |
                          ((uint32_t)motors[addr].irun << 8) |
                           (uint32_t)motors[addr].ihold;
    tmc_write_reg(addr, REG_IHOLD_IRUN, ihold_irun);

    // 3. TPOWERDOWN: Standby power down delay
    tmc_write_reg(addr, REG_TPOWERDOWN, 0x00000004u);

    // 4. CHOPCONF: TOFF=4, HSTRT=5, MRES=6 (4 microsteps), intpol=1
    motors[addr].shadow_chopconf = 0x16000054u;
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);

    // 5. PWMCONF: StealthChop autoscale on
    motors[addr].shadow_pwmconf = 0xC10D0024u;
    tmc_write_reg(addr, REG_PWMCONF, motors[addr].shadow_pwmconf);

    // 6. SGTHRS & TCOOLTHRS: Default StallGuard settings
    tmc_write_reg(addr, REG_SGTHRS, (uint32_t)motors[addr].sg_threshold);
    // TCOOLTHRS = 0xFFFFF ensures StallGuard is active across normal operating speed range
    tmc_write_reg(addr, REG_TCOOLTHRS, 0x000FFFFFu);

    // Verify writes by checking IFCNT change
    bool have_after = tmc_read_reg(addr, REG_IFCNT, &ifcnt_after);
    if (have_before && have_after) {
        uint8_t delta = (uint8_t)(ifcnt_after - ifcnt_before);
        ESP_LOGI(TAG, "Motor [%u] configured. Writes verified (IFCNT: %lu -> %lu, delta: %u)",
                 addr, (unsigned long)ifcnt_before, (unsigned long)ifcnt_after, delta);
    } else {
        ESP_LOGI(TAG, "Motor [%u] configured (writes sent)", addr);
    }

    motors[addr].configured = true;
    return true;
}

void tmc_scan_and_configure_all(void)
{
    ESP_LOGI(TAG, "--- Scanning TMC2209 Bus for Motors 0..3 ---");
    for (uint8_t addr = 0; addr < TMC_NUM_MOTORS; addr++) {
        if (tmc_probe_motor(addr, true)) {
            ESP_LOGI(TAG, "Found TMC2209 at address %u! Configuring...", addr);
            tmc_configure_motor(addr);
        } else {
            ESP_LOGW(TAG, "No response from address %u (MS1/MS2 combination)", addr);
            motors[addr].present = false;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// -----------------------------------------------------------------------------
// StallGuard4 & Sensorless Homing Functions
// -----------------------------------------------------------------------------
bool tmc_stallguard_set_threshold(uint8_t addr, uint8_t threshold)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].sg_threshold = threshold;
    tmc_write_reg(addr, REG_SGTHRS, (uint32_t)threshold);
    ESP_LOGI(TAG, "Motor [%u] StallGuard threshold set to %u", addr, threshold);
    return true;
}

bool tmc_stallguard_get_result(uint8_t addr, uint16_t *out_result)
{
    if (addr >= TMC_NUM_MOTORS || out_result == NULL) return false;
    uint32_t raw_val = 0;
    if (tmc_read_reg(addr, REG_SG_RESULT, &raw_val)) {
        *out_result = (uint16_t)(raw_val & 0x03FFu);  // 10-bit StallGuard result (0..510)
        motors[addr].last_sg_result = *out_result;
        return true;
    }
    return false;
}

bool tmc_stallguard_enable(uint8_t addr, uint8_t threshold, uint32_t tcoolthrs)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].sg_threshold = threshold;
    tmc_write_reg(addr, REG_SGTHRS, (uint32_t)threshold);
    tmc_write_reg(addr, REG_TCOOLTHRS, tcoolthrs);
    ESP_LOGI(TAG, "Motor [%u] StallGuard enabled (SGTHRS=%u, TCOOLTHRS=0x%05lX)",
             addr, threshold, (unsigned long)tcoolthrs);
    return true;
}

bool tmc_stallguard_check_stall(uint8_t addr, bool *is_stalled)
{
    if (addr >= TMC_NUM_MOTORS || is_stalled == NULL) return false;

    // StallGuard4 operates on motor back-EMF, which is zero at standstill or low velocities.
    // Reading SG_RESULT below operational threshold produces 0 by hardware design.
    // Guard against false stall lockouts at standstill or startup:
    if (motors[addr].motion_mode == MOTION_MODE_STEP_DIR) {
        if (motors[addr].target_speed_hz < 1000 || motors[addr].current_speed_hz < 800) {
            motors[addr].is_stalled = false;
            *is_stalled = false;
            return true;
        }
    } else {
        int32_t abs_vel = (motors[addr].vactual_velocity < 0) ? -motors[addr].vactual_velocity : motors[addr].vactual_velocity;
        if (abs_vel < 1000) {
            motors[addr].is_stalled = false;
            *is_stalled = false;
            return true;
        }
    }

    // Check driver standstill status bit
    tmc_drv_status_t st;
    if (tmc_get_drv_status(addr, &st) && st.standstill) {
        motors[addr].is_stalled = false;
        *is_stalled = false;
        return true;
    }

    uint16_t sg_val = 0;
    if (!tmc_stallguard_get_result(addr, &sg_val)) {
        return false;
    }
    
    // TMC2209: Stall occurs when SG_RESULT <= 2 * SGTHRS
    bool stall_detected = (sg_val <= ((uint16_t)motors[addr].sg_threshold * 2));
    motors[addr].is_stalled = stall_detected;
    *is_stalled = stall_detected;

    if (stall_detected) {
        robot_arm_stallguard_event_callback(addr, sg_val);
    }
    return true;
}

void tmc_clear_stall(uint8_t addr)
{
    if (addr < TMC_NUM_MOTORS) {
        motors[addr].is_stalled = false;
    }
}

bool tmc_sensorless_homing(uint8_t addr, int32_t homing_velocity, uint8_t sg_threshold)
{
    if (addr >= TMC_NUM_MOTORS || !motors[addr].present) {
        ESP_LOGE(TAG, "Homing failed: motor %u not available", addr);
        return false;
    }

    ESP_LOGI(TAG, "Starting Sensorless Homing for Motor %u (vel=%ld, sgthrs=%u)...",
             addr, (long)homing_velocity, sg_threshold);

    // 1. Configure StallGuard threshold
    tmc_stallguard_set_threshold(addr, sg_threshold);
    tmc_write_reg(addr, REG_TCOOLTHRS, 0x000FFFFFu);

    // 2. Command motor motion using UART VACTUAL
    tmc_set_velocity_vactual(addr, homing_velocity);

    // Let motor accelerate away from standstill
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. Monitor StallGuard until collision / hard-stop
    bool stalled = false;
    int timeout_ms = 4000;
    while (timeout_ms > 0) {
        if (tmc_stallguard_check_stall(addr, &stalled) && stalled) {
            ESP_LOGW(TAG, "Motor %u Homing Contact Detected! (SG_RESULT=%u)",
                     addr, motors[addr].last_sg_result);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        timeout_ms -= 20;
    }

    // 4. Immediately stop motor
    tmc_stop_vactual(addr);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (stalled) {
        ESP_LOGI(TAG, "Motor %u Homing Complete!", addr);
        return true;
    } else {
        ESP_LOGW(TAG, "Motor %u Homing timed out without detecting stall", addr);
        return false;
    }
}

// -----------------------------------------------------------------------------
// CoolStep (Smart Energy Auto-Current Scaling)
// -----------------------------------------------------------------------------
bool tmc_coolstep_enable(uint8_t addr, uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    // COOLCONF: semin[3:0], seup[6:5], semax[11:8], sedn[14:13], seimin[15]
    uint32_t coolconf = ((uint32_t)(seimin ? 1 : 0) << 15) |
                        ((uint32_t)(sedn & 0x03) << 13)    |
                        ((uint32_t)(semax & 0x0F) << 8)    |
                        ((uint32_t)(seup & 0x03) << 5)     |
                         (uint32_t)(semin & 0x0F);

    tmc_write_reg(addr, REG_COOLCONF, coolconf);
    ESP_LOGI(TAG, "Motor [%u] CoolStep enabled (COOLCONF=0x%05lX)", addr, (unsigned long)coolconf);
    return true;
}

bool tmc_coolstep_disable(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_COOLCONF, 0x00000000u);
    ESP_LOGI(TAG, "Motor [%u] CoolStep disabled", addr);
    return true;
}

// -----------------------------------------------------------------------------
// Current Scaling, Microstepping & Chopper Mode
// -----------------------------------------------------------------------------
bool tmc_set_current(uint8_t addr, uint8_t irun, uint8_t ihold, uint8_t iholddelay)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    if (irun > 31) irun = 31;
    if (ihold > 31) ihold = 31;
    if (iholddelay > 15) iholddelay = 15;

    motors[addr].irun = irun;
    motors[addr].ihold = ihold;
    motors[addr].iholddelay = iholddelay;

    uint32_t val = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
    tmc_write_reg(addr, REG_IHOLD_IRUN, val);
    ESP_LOGI(TAG, "Motor [%u] Current: IRUN=%u, IHOLD=%u, DELAY=%u", addr, irun, ihold, iholddelay);
    return true;
}

bool tmc_set_microsteps(uint8_t addr, uint16_t microsteps, bool interpolate)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    uint8_t mres = TMC_MRES_4;
    switch (microsteps) {
        case 256: mres = TMC_MRES_256; break;
        case 128: mres = TMC_MRES_128; break;
        case 64:  mres = TMC_MRES_64;  break;
        case 32:  mres = TMC_MRES_32;  break;
        case 16:  mres = TMC_MRES_16;  break;
        case 8:   mres = TMC_MRES_8;   break;
        case 4:   mres = TMC_MRES_4;   break;
        case 2:   mres = TMC_MRES_2;   break;
        case 1:   mres = TMC_MRES_1;   break;
        default:
            ESP_LOGW(TAG, "Invalid microstep %u, defaulting to 16", microsteps);
            mres = TMC_MRES_16;
            break;
    }

    motors[addr].microsteps = microsteps;
    motors[addr].shadow_chopconf &= ~(0x1Fu << 24); // Clear MRES and intpol
    motors[addr].shadow_chopconf |= ((uint32_t)(mres & 0x0F) << 24);
    if (interpolate) {
        motors[addr].shadow_chopconf |= (1u << 28);
    }
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);
    ESP_LOGI(TAG, "Motor [%u] Microsteps set to 1/%u (MRES=%u, intpol=%d)",
             addr, microsteps, mres, interpolate);
    return true;
}

bool tmc_set_spreadcycle(uint8_t addr, bool enable_spreadcycle)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].spreadcycle_en = enable_spreadcycle;
    if (enable_spreadcycle) {
        motors[addr].shadow_gconf |= (1u << 2);
    } else {
        motors[addr].shadow_gconf &= ~(1u << 2);
    }
    tmc_write_reg(addr, REG_GCONF, motors[addr].shadow_gconf);
    ESP_LOGI(TAG, "Motor [%u] Chopper mode: %s", addr, enable_spreadcycle ? "SpreadCycle" : "StealthChop");
    return true;
}

bool tmc_set_stealthchop_pwm_threshold(uint8_t addr, uint32_t tpwmthrs)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_TPWMTHRS, tpwmthrs);
    return true;
}

// -----------------------------------------------------------------------------
// Motion & Velocity Control (UART VACTUAL and STEP/DIR)
// -----------------------------------------------------------------------------
bool tmc_set_velocity_vactual(uint8_t addr, int32_t velocity)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].motion_mode = MOTION_MODE_UART_VACTUAL;
    motors[addr].vactual_velocity = velocity;
    tmc_write_reg(addr, REG_VACTUAL, (uint32_t)velocity);
    return true;
}

bool tmc_stop_vactual(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].vactual_velocity = 0;
    tmc_write_reg(addr, REG_VACTUAL, 0x00000000u);
    // Setting VACTUAL=0 restores STEP/DIR control
    motors[addr].motion_mode = MOTION_MODE_STEP_DIR;
    return true;
}

void tmc_set_motor_speed_hz(uint8_t addr, int speed_hz)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].target_speed_hz = speed_hz;
    motors[addr].is_stalled = false;
}

void tmc_set_motor_direction(uint8_t addr, bool forward)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].dir_level = forward ? 1 : 0;
    gpio_set_level(motors[addr].dir_pin, motors[addr].dir_level);
}

void tmc_set_motor_enable(uint8_t addr, bool enable)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].enabled = enable;
    gpio_set_level(motors[addr].en_pin, enable ? 0 : 1); // Active LOW
    tmc_set_software_enable(addr, enable);
}

bool tmc_set_shaft_direction_uart(uint8_t addr, bool reverse)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].reverse_dir = reverse;
    if (reverse) {
        motors[addr].shadow_gconf |= (1u << 3);
    } else {
        motors[addr].shadow_gconf &= ~(1u << 3);
    }
    tmc_write_reg(addr, REG_GCONF, motors[addr].shadow_gconf);
    return true;
}

bool tmc_set_software_enable(uint8_t addr, bool enable)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    // TOFF in CHOPCONF bits 0..3: 0 = disabled bridge, 4 = normal chopper active
    if (enable) {
        motors[addr].shadow_chopconf = (motors[addr].shadow_chopconf & ~0x0Fu) | 0x04u;
    } else {
        motors[addr].shadow_chopconf &= ~0x0Fu;
    }
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);
    return true;
}

bool tmc_set_freewheel_mode(uint8_t addr, uint8_t mode)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    // PWMCONF freewheel: bits 14..15
    motors[addr].shadow_pwmconf &= ~(0x03u << 14);
    motors[addr].shadow_pwmconf |= ((uint32_t)(mode & 0x03) << 14);
    tmc_write_reg(addr, REG_PWMCONF, motors[addr].shadow_pwmconf);
    return true;
}

bool tmc_set_standby_delay(uint8_t addr, uint8_t delay)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_TPOWERDOWN, (uint32_t)delay);
    return true;
}

// -----------------------------------------------------------------------------
// Multi-Motor Simultaneous Controls
// -----------------------------------------------------------------------------
void tmc_run_all_motors(int speed_hz)
{
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            motors[i].target_speed_hz = speed_hz;
            gpio_set_level(motors[i].en_pin, 0);
        }
    }
}

void tmc_stop_all_motors(void)
{
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].target_speed_hz = 0;
        motors[i].current_speed_hz = 0;
        motors[i].is_stalled = false;
        if (motors[i].motion_mode == MOTION_MODE_UART_VACTUAL) {
            tmc_stop_vactual(i);
        }
    }
}

void tmc_emergency_stop_all(void)
{
    // Cut software bridges & set speed to 0 immediately
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].target_speed_hz = 0;
        motors[i].current_speed_hz = 0;
        gpio_set_level(motors[i].en_pin, 1);     // Disable EN pin (HIGH)
        tmc_write_reg(i, REG_VACTUAL, 0);        // Zero velocity
        tmc_set_software_enable(i, false);       // Kill bridge
    }
    ESP_LOGE(TAG, "EMERGENCY STOP EXECUTED ON ALL 4 MOTORS!");
}

// -----------------------------------------------------------------------------
// Telemetry & Diagnostics API
// -----------------------------------------------------------------------------
bool tmc_get_drv_status(uint8_t addr, tmc_drv_status_t *status)
{
    if (addr >= TMC_NUM_MOTORS || status == NULL) return false;
    uint32_t reg_val = 0;
    if (!tmc_read_reg(addr, REG_DRV_STATUS, &reg_val)) {
        return false;
    }

    status->addr         = addr;
    status->raw_status   = reg_val;
    status->otpw         = (reg_val >> 0) & 1u;
    status->ot           = (reg_val >> 1) & 1u;
    status->s2ga         = (reg_val >> 2) & 1u;
    status->s2gb         = (reg_val >> 3) & 1u;
    status->ola          = (reg_val >> 6) & 1u;
    status->olb          = (reg_val >> 7) & 1u;
    status->cs_actual    = (reg_val >> 16) & 0x1Fu;
    status->stealthchop  = (reg_val >> 30) & 1u;
    status->standstill   = (reg_val >> 31) & 1u;

    uint16_t sg = 0;
    tmc_stallguard_get_result(addr, &sg);
    status->stallguard_result = sg;

    return true;
}

bool tmc_get_ioin(uint8_t addr, uint32_t *ioin_val)
{
    return tmc_read_reg(addr, REG_IOIN, ioin_val);
}

bool tmc_clear_gstat(uint8_t addr)
{
    // Write 1 to clear flags in GSTAT
    tmc_write_reg(addr, REG_GSTAT, 0x00000007u);
    return true;
}

void tmc_print_motor_status(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return;
    if (!motors[addr].present) {
        printf("[M%u] Not detected on bus\n", addr);
        return;
    }

    tmc_drv_status_t st;
    if (!tmc_get_drv_status(addr, &st)) {
        printf("[M%u] Status read failed\n", addr);
        return;
    }

    printf("[M%u Addr %u] Target: %5dHz | Actual: %5dHz | CS: %2u | SG4: %3u |",
           addr, addr, motors[addr].target_speed_hz, motors[addr].current_speed_hz,
           st.cs_actual, st.stallguard_result);

    if (st.standstill) printf(" [STANDSTILL]");
    if (st.otpw)       printf(" [OTPW]");
    if (st.ot)         printf(" [OVERTEMP]");
    if (st.s2ga)       printf(" [S2GA]");
    if (st.s2gb)       printf(" [S2GB]");
    if (st.ola)        printf(" [OPEN_A]");
    if (st.olb)        printf(" [OPEN_B]");
    if (motors[addr].is_stalled) printf(" [STALL_DETECTED]");
    if (!(st.otpw | st.ot | st.s2ga | st.s2gb | st.ola | st.olb | st.standstill)) {
        printf(" [OK]");
    }
    printf("\n");
}

void tmc_print_all_motors_status(void)
{
    printf("==================== 4-MOTOR TELEMETRY ====================\n");
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        tmc_print_motor_status(i);
    }
    printf("===========================================================\n");
    fflush(stdout);
}

bool tmc_get_position_tracking_error(uint8_t addr, float *out_error_deg)
{
    if (addr >= TMC_NUM_MOTORS) return false;

    // Calculate theoretical position from steps
    float steps_per_motor_rev = 200.0f * (float)motors[addr].microsteps;
    if (steps_per_motor_rev <= 0.0f) steps_per_motor_rev = 200.0f;

    float motor_turns = (float)motors[addr].total_commanded_steps / steps_per_motor_rev;
    motors[addr].commanded_motor_deg = motor_turns * 360.0f;
    // 20:1 reduction with inverted direction
    motors[addr].commanded_output_deg = -(motors[addr].commanded_motor_deg / 20.0f);
    motors[addr].tracking_error_deg = 0.0f;

    if (out_error_deg != NULL) {
        *out_error_deg = 0.0f;
    }
    return true;
}

void tmc_reset_position(uint8_t addr)
{
    if (addr < TMC_NUM_MOTORS) {
        motors[addr].total_commanded_steps = 0;
        motors[addr].commanded_motor_deg = 0.0f;
        motors[addr].commanded_output_deg = 0.0f;
        motors[addr].tracking_error_deg = 0.0f;
    }
}

// -----------------------------------------------------------------------------
// Robotics Kinematics & Trajectory Hooks (Dummy / Extensible Callbacks)
// -----------------------------------------------------------------------------
void robot_arm_joint_move_dummy(uint8_t joint_id, float target_angle_deg, float speed_deg_s)
{
    if (joint_id >= TMC_NUM_MOTORS) return;

    float current_out_deg = motors[joint_id].commanded_output_deg;
    ESP_LOGI("ROBOT_DUMMY", "Joint %u commanded to %.2f deg (current=%.2f deg, err=%.2f deg) at %.2f deg/s",
             joint_id, target_angle_deg, current_out_deg,
             target_angle_deg - current_out_deg, speed_deg_s);

    // Convert output degrees/sec to motor step frequency through 20:1 gearbox:
    // Output 1 deg requires motor to spin 20 deg.
    float steps_per_motor_rev = 200.0f * (float)motors[joint_id].microsteps;
    float steps_per_out_deg = (steps_per_motor_rev * 20.0f) / 360.0f;
    int steps_per_sec = (int)(speed_deg_s * steps_per_out_deg);
    if (steps_per_sec < 100) steps_per_sec = 100;
    if (steps_per_sec > 12000) steps_per_sec = 12000;

    tmc_set_motor_speed_hz(joint_id, steps_per_sec);
}

void robot_arm_stream_trajectory_dummy(const float joint_angles[4], const float joint_speeds[4])
{
    ESP_LOGI("ROBOT_DUMMY", "Streaming 4-DOF Trajectory: J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f",
             joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]);

    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            robot_arm_joint_move_dummy(i, joint_angles[i], joint_speeds[i]);
        }
    }
}

void robot_arm_stallguard_event_callback(uint8_t joint_id, uint16_t sg_result)
{
    ESP_LOGW("ROBOT_SAFETY", ">>> STALLGUARD EVENT: Joint %u obstacle/contact detected (SG_RESULT=%u)!",
             joint_id, sg_result);
    // Automatic safety action: Stop this specific stalled joint
    motors[joint_id].target_speed_hz = 0;
    if (motors[joint_id].motion_mode == MOTION_MODE_UART_VACTUAL) {
        tmc_stop_vactual(joint_id);
    }
}

void robot_arm_sync_home_all_dummy(void)
{
    ESP_LOGI("ROBOT_DUMMY", "Triggering sensorless homing sequence for all detected joints...");
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            tmc_sensorless_homing(i, 2000, 64);
        }
    }
}

