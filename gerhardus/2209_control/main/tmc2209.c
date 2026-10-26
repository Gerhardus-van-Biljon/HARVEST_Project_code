/*
 * tmc2209 driver implementation for esp32-s3.
 * handles the 5 motors across our two uart buses, sets up chopper modes,
 * handles current scaling, stallguard thresholds, and position math with as5600.
 */

#include "tmc2209.h"
#include "as5600.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/uart.h"

static const char *TAG = "TMC2209";

// uart ports and pin map
#define TMC_BUS0_PORT       UART_NUM_1
#define TMC_BUS1_PORT       UART_NUM_2
#define TMC_BUS0_TX_PIN     GPIO_NUM_10     // goes to driver rx via 1k resistor
#define TMC_BUS0_RX_PIN     GPIO_NUM_11     // comes from driver tx
#define TMC_BUS1_TX_PIN     GPIO_NUM_16     // goes to motor 4 rx via 1k resistor
#define TMC_BUS1_RX_PIN     GPIO_NUM_21     // comes from motor 4 tx

// pin assignments for the 5 motors on esp32-s3
// motor 0: address 0 (ms1=gnd, ms2=gnd)
#define MOTOR0_STEP_PIN   GPIO_NUM_9
#define MOTOR0_DIR_PIN    GPIO_NUM_2
#define MOTOR0_EN_PIN     GPIO_NUM_4

// motor 1: address 1 (ms1=vio, ms2=gnd)
#define MOTOR1_STEP_PIN   GPIO_NUM_5
#define MOTOR1_DIR_PIN    GPIO_NUM_6
#define MOTOR1_EN_PIN     GPIO_NUM_4

// motor 2: address 2 (ms1=gnd, ms2=vio)
#define MOTOR2_STEP_PIN   GPIO_NUM_7
#define MOTOR2_DIR_PIN    GPIO_NUM_8
#define MOTOR2_EN_PIN     GPIO_NUM_4

// motor 3: address 3 (ms1=vio, ms2=vio)
#define MOTOR3_STEP_PIN   GPIO_NUM_12
#define MOTOR3_DIR_PIN    GPIO_NUM_13
#define MOTOR3_EN_PIN     GPIO_NUM_4

// motor 4: separate bus 1, addr 0 (ms1=gnd, ms2=gnd)
#define MOTOR4_STEP_PIN   GPIO_NUM_14
#define MOTOR4_DIR_PIN    GPIO_NUM_15
#define MOTOR4_EN_PIN     GPIO_NUM_4

// global state array for our 5 motors
tmc_motor_t motors[TMC_NUM_MOTORS];

// trinamic 8-bit crc using polynomial x^8 + x^2 + x + 1 (seed 0x07)
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

// set up both uart hardware buses: uart1 for motors 0..3, uart2 for motor 4
esp_err_t tmc_uart_bus_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // bus 0 (uart1) covers motors 0 through 3
    esp_err_t err = uart_param_config(TMC_BUS0_PORT, &uart_cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(TMC_BUS0_PORT, TMC_BUS0_TX_PIN, TMC_BUS0_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(TMC_BUS0_PORT, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;

    // bus 1 (uart2) handles motor 4 on its own lines
    err = uart_param_config(TMC_BUS1_PORT, &uart_cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(TMC_BUS1_PORT, TMC_BUS1_TX_PIN, TMC_BUS1_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(TMC_BUS1_PORT, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Dual UART buses initialized: Bus 0 (UART%d TX=%d RX=%d), Bus 1 (UART%d TX=%d RX=%d)",
             TMC_BUS0_PORT, TMC_BUS0_TX_PIN, TMC_BUS0_RX_PIN,
             TMC_BUS1_PORT, TMC_BUS1_TX_PIN, TMC_BUS1_RX_PIN);
    return ESP_OK;
}

// sends an 8-byte write datagram to a specific uart port and bus address
void tmc_write_reg_bus(int uart_port, uint8_t bus_addr, uint8_t reg, uint32_t value)
{
    uint8_t datagram[8] = {0};

    datagram[0] = 0x05;                         // trinamic sync byte
    datagram[1] = bus_addr;                     // slave address (0..3)
    datagram[2] = (uint8_t)(reg | 0x80u);       // register address with write bit set (msb=1)
    datagram[3] = (uint8_t)((value >> 24) & 0xFFu);
    datagram[4] = (uint8_t)((value >> 16) & 0xFFu);
    datagram[5] = (uint8_t)((value >> 8) & 0xFFu);
    datagram[6] = (uint8_t)(value & 0xFFu);

    tmc_calc_crc(datagram, sizeof(datagram));

    uart_write_bytes(uart_port, (const char *)datagram, sizeof(datagram));
    uart_wait_tx_done(uart_port, pdMS_TO_TICKS(10));
    uart_flush_input(uart_port);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// helper to write register using logical motor index
void tmc_write_reg(uint8_t addr, uint8_t reg, uint32_t value)
{
    if (addr >= TMC_NUM_MOTORS) return;
    int port = (motors[addr].uart_port != 0) ? motors[addr].uart_port : ((addr < 4) ? TMC_BUS0_PORT : TMC_BUS1_PORT);
    uint8_t bus_addr = (motors[addr].uart_port != 0) ? motors[addr].uart_addr : ((addr < 4) ? addr : 0);
    tmc_write_reg_bus(port, bus_addr, reg, value);
}

// reads a register from the driver (4 byte request, 8 byte response packet)
bool tmc_read_reg_bus(int uart_port, uint8_t bus_addr, uint8_t reg, uint32_t *out, bool verbose)
{
    uint8_t req[4] = {0x05, bus_addr, reg, 0};
    tmc_calc_crc(req, sizeof(req));

    uint8_t buf[16];
    uart_flush_input(uart_port);
    uart_write_bytes(uart_port, (const char *)req, sizeof(req));
    uart_wait_tx_done(uart_port, pdMS_TO_TICKS(10));

    int len = uart_read_bytes(uart_port, buf, sizeof(buf), pdMS_TO_TICKS(20));

    if (verbose) {
        printf("[UART %d] Probing Addr %u | TX: %02X %02X %02X %02X | RX (%d bytes):",
               uart_port, bus_addr, req[0], req[1], req[2], req[3], len);
        for (int i = 0; i < len; i++) printf(" %02X", buf[i]);
    }

    for (int i = 0; i + 8 <= len; i++) {
        // sync byte is 0x05, master reply address is 0xFF, reg address must match
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
            if (verbose) printf(" | VALID reply from Addr %u! Val=0x%08lX\n", bus_addr, (unsigned long)(out ? *out : 0));
            return true;
        }
    }

    if (verbose) printf(" | No reply\n");
    return false;
}

// reads register for a logical motor index, optional verbose debugging print
bool tmc_read_reg_ex(uint8_t addr, uint8_t reg, uint32_t *out, bool verbose)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    int port = (motors[addr].uart_port != 0) ? motors[addr].uart_port : ((addr < 4) ? TMC_BUS0_PORT : TMC_BUS1_PORT);
    uint8_t bus_addr = (motors[addr].uart_port != 0) ? motors[addr].uart_addr : ((addr < 4) ? addr : 0);
    return tmc_read_reg_bus(port, bus_addr, reg, out, verbose);
}

// standard quiet register read
bool tmc_read_reg(uint8_t addr, uint8_t reg, uint32_t *out)
{
    return tmc_read_reg_ex(addr, reg, out, false);
}

// initialize motor structs with pinouts, addresses, and current limits
void tmc_init_motor_structures(void)
{
    // motor 0: bus 0, addr 0 (ms1=gnd, ms2=gnd)
    // 60mm bipolar 2A nema 17 for the heavy base joint
    motors[0].motor_id  = 0;
    motors[0].bus_id    = 0;
    motors[0].uart_port = TMC_BUS0_PORT;
    motors[0].uart_addr = 0;
    motors[0].addr      = 0;
    motors[0].step_pin  = MOTOR0_STEP_PIN;
    motors[0].dir_pin   = MOTOR0_DIR_PIN;
    motors[0].en_pin    = MOTOR0_EN_PIN;
    motors[0].irun      = 25;                  // ~1.44A RMS / 2.03A peak for 2.0A motor
    motors[0].ihold     = 12;                  // ~0.72A RMS holding current

    // motor 1: bus 0, addr 1 (ms1=vio, ms2=gnd)
    // 17HS3401S nema 17 (1.3A peak / 0.92A RMS rated)
    motors[1].motor_id  = 1;
    motors[1].bus_id    = 0;
    motors[1].uart_port = TMC_BUS0_PORT;
    motors[1].uart_addr = 1;
    motors[1].addr      = 1;
    motors[1].step_pin  = MOTOR1_STEP_PIN;
    motors[1].dir_pin   = MOTOR1_DIR_PIN;
    motors[1].en_pin    = MOTOR1_EN_PIN;
    motors[1].irun      = 16;                  // ~0.94A RMS / 1.33A peak (keeps temps sensible)
    motors[1].ihold     = 8;                   // ~0.50A RMS holding current

    // motor 2: bus 0, addr 2 (ms1=gnd, ms2=vio)
    // 17HS3401S nema 17
    motors[2].motor_id  = 2;
    motors[2].bus_id    = 0;
    motors[2].uart_port = TMC_BUS0_PORT;
    motors[2].uart_addr = 2;
    motors[2].addr      = 2;
    motors[2].step_pin  = MOTOR2_STEP_PIN;
    motors[2].dir_pin   = MOTOR2_DIR_PIN;
    motors[2].en_pin    = MOTOR2_EN_PIN;
    motors[2].irun      = 16;
    motors[2].ihold     = 8;

    // motor 3: bus 0, addr 3 (ms1=vio, ms2=vio)
    // 17HS3401S nema 17
    motors[3].motor_id  = 3;
    motors[3].bus_id    = 0;
    motors[3].uart_port = TMC_BUS0_PORT;
    motors[3].uart_addr = 3;
    motors[3].addr      = 3;
    motors[3].step_pin  = MOTOR3_STEP_PIN;
    motors[3].dir_pin   = MOTOR3_DIR_PIN;
    motors[3].en_pin    = MOTOR3_EN_PIN;
    motors[3].irun      = 16;
    motors[3].ihold     = 8;

    // motor 4: bus 1, addr 0 (ms1=gnd, ms2=gnd on uart2)
    // 17HS3401S nema 17
    motors[4].motor_id  = 4;
    motors[4].bus_id    = 1;
    motors[4].uart_port = TMC_BUS1_PORT;
    motors[4].uart_addr = 0;
    motors[4].addr      = 4;
    motors[4].step_pin  = MOTOR4_STEP_PIN;
    motors[4].dir_pin   = MOTOR4_DIR_PIN;
    motors[4].en_pin    = MOTOR4_EN_PIN;
    motors[4].irun      = 16;
    motors[4].ihold     = 8;

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
        motors[i].sg_threshold = 64;           // default stallguard threshold
        motors[i].last_sg_result = 0;
        motors[i].is_stalled = false;
        motors[i].iholddelay = 1;
        motors[i].microsteps = 4;              // 4 microsteps
        motors[i].reverse_dir = false;
        motors[i].spreadcycle_en = false;      // stealthchop on (needed for stallguard4 & silence)

        motors[i].total_commanded_steps = 0;
        motors[i].commanded_motor_deg   = 0.0f;
        motors[i].commanded_output_deg  = 0.0f;
        motors[i].tracking_error_deg    = 0.0f;
        motors[i].tracking_error_motor_deg = 0.0f;

        // shadow register defaults: bit2=0 stealthchop, bit6=1 pdn_disable, bit7=1 mstep_reg_select
        motors[i].shadow_gconf    = 0x000000C0u;
        motors[i].shadow_chopconf = 0x16000054u;  // toff=4, hstrt=5, mres=6 (4 mstep), intpol=1
        motors[i].shadow_pwmconf  = 0xC10D0024u;  // stealthchop autoscale on
    }
}

// configure gpio directions and default levels for step, dir, enable
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

    // set initial pin states
    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        gpio_set_level(motors[i].en_pin, 0);     // active low = enabled
        gpio_set_level(motors[i].dir_pin, motors[i].dir_level);
        gpio_set_level(motors[i].step_pin, 0);
    }
}

// test if driver answers on the bus by reading IFCNT
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

// configure registers: chopper settings, current limits, stealthchop, stallguard thresholds
bool tmc_configure_motor(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;

    uint32_t ifcnt_before = 0, ifcnt_after = 0;
    bool have_before = tmc_read_reg(addr, REG_IFCNT, &ifcnt_before);

    // 1. gconf: bit2 spreadcycle (0=stealthchop, 1=spreadcycle), bit6 pdn_disable, bit7 mstep_reg_select
    // default to stealthchop (bit2=0) which stallguard4 requires on tmc2209
    motors[addr].shadow_gconf = 0x000000C0u;
    if (motors[addr].reverse_dir) {
        motors[addr].shadow_gconf |= (1u << 3);  // shaft bit
    }
    if (motors[addr].spreadcycle_en) {
        motors[addr].shadow_gconf |= (1u << 2);  // spreadcycle bit
    }
    tmc_write_reg(addr, REG_GCONF, motors[addr].shadow_gconf);

    // 2. ihold_irun: ihold[4:0], irun[12:8], iholddelay[19:16]
    uint32_t ihold_irun = ((uint32_t)motors[addr].iholddelay << 16) |
                          ((uint32_t)motors[addr].irun << 8) |
                           (uint32_t)motors[addr].ihold;
    tmc_write_reg(addr, REG_IHOLD_IRUN, ihold_irun);

    // 3. tpowerdown: standby power down delay
    tmc_write_reg(addr, REG_TPOWERDOWN, 0x00000004u);

    // 4. chopconf: toff=4, hstrt=5, mres=6 (4 microsteps), intpol=1
    motors[addr].shadow_chopconf = 0x16000054u;
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);

    // 5. pwmconf: turn on stealthchop autoscale
    motors[addr].shadow_pwmconf = 0xC10D0024u;
    tmc_write_reg(addr, REG_PWMCONF, motors[addr].shadow_pwmconf);

    // 6. sgthrs & tcoolthrs: stallguard setup
    tmc_write_reg(addr, REG_SGTHRS, (uint32_t)motors[addr].sg_threshold);
    // tcoolthrs = 0xfffff ensures stallguard stays active across our working speeds
    tmc_write_reg(addr, REG_TCOOLTHRS, 0x000FFFFFu);

    // check if writes landed by comparing ifcnt counter
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

// scan all 5 motors across both buses and configure any that reply
void tmc_scan_and_configure_all(void)
{
    ESP_LOGI(TAG, "--- Scanning Dual TMC2209 Buses for Motors 0..4 ---");
    for (uint8_t addr = 0; addr < TMC_NUM_MOTORS; addr++) {
        ESP_LOGI(TAG, "Probing Motor [%u] on UART Bus %u (Port UART%d), Bus Addr %u...",
                 addr, motors[addr].bus_id, motors[addr].uart_port, motors[addr].uart_addr);
        if (tmc_probe_motor(addr, true)) {
            ESP_LOGI(TAG, "Found TMC2209 for Motor [%u]! Configuring...", addr);
            tmc_configure_motor(addr);
        } else {
            ESP_LOGW(TAG, "No response from Motor [%u] (Bus %u, Addr %u)",
                     addr, motors[addr].bus_id, motors[addr].uart_addr);
            motors[addr].present = false;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// update stallguard threshold register
bool tmc_stallguard_set_threshold(uint8_t addr, uint8_t threshold)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].sg_threshold = threshold;
    tmc_write_reg(addr, REG_SGTHRS, (uint32_t)threshold);
    ESP_LOGI(TAG, "Motor [%u] StallGuard threshold set to %u", addr, threshold);
    return true;
}

// read back current stallguard result (10 bits, 0..510)
bool tmc_stallguard_get_result(uint8_t addr, uint16_t *out_result)
{
    if (addr >= TMC_NUM_MOTORS || out_result == NULL) return false;
    uint32_t raw_val = 0;
    if (tmc_read_reg(addr, REG_SG_RESULT, &raw_val)) {
        *out_result = (uint16_t)(raw_val & 0x03FFu);
        motors[addr].last_sg_result = *out_result;
        return true;
    }
    return false;
}

// turn on stallguard by writing threshold and lower velocity limit
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

// check if motor has hit an obstacle or stalled
// note: only valid when moving fast enoug at standstill back-emf is 0 so sg is naturally 0
bool tmc_stallguard_check_stall(uint8_t addr, bool *is_stalled)
{
    if (addr >= TMC_NUM_MOTORS || is_stalled == NULL) return false;
    *is_stalled = false;

    //  motor is stopped so stallguard isn't active
    if (motors[addr].target_speed_hz <= 0 && motors[addr].vactual_velocity == 0) {
        motors[addr].is_stalled = false;
        return true;
    }

    //  step rate too low (<400 hz): back-emf isn't strong enough for stallguard4
    if (motors[addr].current_speed_hz < 400 && motors[addr].vactual_velocity == 0) {
        motors[addr].is_stalled = false;
        return true;
    }

    //  check driver standstill flag
    tmc_drv_status_t st;
    if (!tmc_get_drv_status(addr, &st)) {
        return false;
    }

    // at standstill hardware disables stallguard and returns 0, ignore it
    if (st.standstill) {
        motors[addr].is_stalled = false;
        return true;
    }

    uint16_t sg_val = st.stallguard_result;

    // 4. in stealthchop mode, back-emf drops when load increases
    // stall triggers when sg_result drops to or below 2 * sgthrs
    bool stall_detected = (sg_val <= ((uint16_t)motors[addr].sg_threshold * 2));
    motors[addr].is_stalled = stall_detected;
    *is_stalled = stall_detected;

    if (stall_detected) {
        robot_arm_stallguard_event_callback(addr, sg_val);
    }
    return true;
}

// sensorless homing: moves motor until stallguard detects a hard stop
bool tmc_sensorless_homing(uint8_t addr, int32_t homing_velocity, uint8_t sg_threshold)
{
    if (addr >= TMC_NUM_MOTORS || !motors[addr].present) {
        ESP_LOGE(TAG, "Homing failed: motor %u not available", addr);
        return false;
    }

    ESP_LOGI(TAG, "Starting Sensorless Homing for Motor %u (vel=%ld, sgthrs=%u)...",
             addr, (long)homing_velocity, sg_threshold);

    // 1. set stallguard sensitivity
    tmc_stallguard_set_threshold(addr, sg_threshold);
    tmc_write_reg(addr, REG_TCOOLTHRS, 0x000FFFFFu);

    // 2. spin motor via uart vactual
    tmc_set_velocity_vactual(addr, homing_velocity);

    // let motor get past standstill
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. monitor stallguard until it hits the end stop
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

    // 4. kill velocity immediately
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

// coolstep dynamic current reduction setup
bool tmc_coolstep_enable(uint8_t addr, uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    uint32_t coolconf = ((uint32_t)(seimin ? 1 : 0) << 15) |
                        ((uint32_t)(sedn & 0x03) << 13)    |
                        ((uint32_t)(semax & 0x0F) << 8)    |
                        ((uint32_t)(seup & 0x03) << 5)     |
                         (uint32_t)(semin & 0x0F);

    tmc_write_reg(addr, REG_COOLCONF, coolconf);
    ESP_LOGI(TAG, "Motor [%u] CoolStep enabled (COOLCONF=0x%05lX)", addr, (unsigned long)coolconf);
    return true;
}

// disable coolstep
bool tmc_coolstep_disable(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_COOLCONF, 0x00000000u);
    ESP_LOGI(TAG, "Motor [%u] CoolStep disabled", addr);
    return true;
}

// set run current, hold current, and hold delay
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

// microstepping resolution config (1 to 256) with optional 256 interpolation
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
    motors[addr].shadow_chopconf &= ~(0x1Fu << 24); // clear mres and intpol
    motors[addr].shadow_chopconf |= ((uint32_t)(mres & 0x0F) << 24);
    if (interpolate) {
        motors[addr].shadow_chopconf |= (1u << 28);
    }
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);
    ESP_LOGI(TAG, "Motor [%u] Microsteps set to 1/%u (MRES=%u, intpol=%d)",
             addr, microsteps, mres, interpolate);
    return true;
}

// switch between spreadcycle and stealthchop
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

// set velocity threshold above which driver switches from stealthchop to spreadcycle
bool tmc_set_stealthchop_pwm_threshold(uint8_t addr, uint32_t tpwmthrs)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_TPWMTHRS, tpwmthrs);
    return true;
}

// spin motor directly via internal uart velocity generator
bool tmc_set_velocity_vactual(uint8_t addr, int32_t velocity)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].motion_mode = MOTION_MODE_UART_VACTUAL;
    motors[addr].vactual_velocity = velocity;
    tmc_write_reg(addr, REG_VACTUAL, (uint32_t)velocity);
    return true;
}

// stop internal uart velocity generator and return control to step/dir pins
bool tmc_stop_vactual(uint8_t addr)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].vactual_velocity = 0;
    tmc_write_reg(addr, REG_VACTUAL, 0x00000000u);
    motors[addr].motion_mode = MOTION_MODE_STEP_DIR;
    return true;
}

// set target step rate for the timer isr
void tmc_set_motor_speed_hz(uint8_t addr, int speed_hz)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].target_speed_hz = speed_hz;
    if (speed_hz > 0) {
        motors[addr].is_stalled = false; // reset stall flag when movement is requested
    }
}

// set direction gpio level
void tmc_set_motor_direction(uint8_t addr, bool forward)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].dir_level = forward ? 1 : 0;
    gpio_set_level(motors[addr].dir_pin, motors[addr].dir_level);
}

// enable or disable motor via hardware pin and software register
void tmc_set_motor_enable(uint8_t addr, bool enable)
{
    if (addr >= TMC_NUM_MOTORS) return;
    motors[addr].enabled = enable;
    gpio_set_level(motors[addr].en_pin, enable ? 0 : 1); // active low
    tmc_set_software_enable(addr, enable);
}

// flip shaft rotation direction bit in register
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

// enable or disable power bridge through chopconf toff bits
bool tmc_set_software_enable(uint8_t addr, bool enable)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    // toff: 0 = bridge off, 4 = normal chopper active
    if (enable) {
        motors[addr].shadow_chopconf = (motors[addr].shadow_chopconf & ~0x0Fu) | 0x04u;
    } else {
        motors[addr].shadow_chopconf &= ~0x0Fu;
    }
    tmc_write_reg(addr, REG_CHOPCONF, motors[addr].shadow_chopconf);
    return true;
}

// configure freewheeling mode when standstill occurs
bool tmc_set_freewheel_mode(uint8_t addr, uint8_t mode)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    motors[addr].shadow_pwmconf &= ~(0x03u << 14);
    motors[addr].shadow_pwmconf |= ((uint32_t)(mode & 0x03) << 14);
    tmc_write_reg(addr, REG_PWMCONF, motors[addr].shadow_pwmconf);
    return true;
}

// delay time before switching to hold current
bool tmc_set_standby_delay(uint8_t addr, uint8_t delay)
{
    if (addr >= TMC_NUM_MOTORS) return false;
    tmc_write_reg(addr, REG_TPOWERDOWN, (uint32_t)delay);
    return true;
}

// run all detected motors at a given step rate
void tmc_run_all_motors(int speed_hz)
{
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            motors[i].target_speed_hz = speed_hz;
            gpio_set_level(motors[i].en_pin, 0);
        }
    }
}

// command all motors to stop
void tmc_stop_all_motors(void)
{
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].target_speed_hz = 0;
        motors[i].current_speed_hz = 0;
        if (motors[i].motion_mode == MOTION_MODE_UART_VACTUAL) {
            tmc_stop_vactual(i);
        }
    }
}

// emergency stop: kill steps, disable enable pins, and shut off power bridges
void tmc_emergency_stop_all(void)
{
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].target_speed_hz = 0;
        motors[i].current_speed_hz = 0;
        gpio_set_level(motors[i].en_pin, 1);     // disable en pin (high)
        tmc_write_reg(i, REG_VACTUAL, 0);        // zero velocity
        tmc_set_software_enable(i, false);       // kill bridge
    }
    ESP_LOGE(TAG, "EMERGENCY STOP EXECUTED ON ALL 5 MOTORS!");
}

// read driver status register and unpack flags
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

// read inputs register (uart address pins, diag, etc)
bool tmc_get_ioin(uint8_t addr, uint32_t *ioin_val)
{
    return tmc_read_reg(addr, REG_IOIN, ioin_val);
}

// clear error latch bits in gstat
bool tmc_clear_gstat(uint8_t addr)
{
    tmc_write_reg(addr, REG_GSTAT, 0x00000007u);
    return true;
}

// print status flags for one motor
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

    printf("[M%u Bus%u Addr%u] Target: %5dHz | Actual: %5dHz | CS: %2u | SG4: %3u |",
           addr, motors[addr].bus_id, motors[addr].uart_addr,
           motors[addr].target_speed_hz, motors[addr].current_speed_hz,
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

// dump telemetry for all 5 motors
void tmc_print_all_motors_status(void)
{
    printf("==================== 5-MOTOR TELEMETRY ====================\n");
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        tmc_print_motor_status(i);
    }
    printf("===========================================================\n");
    fflush(stdout);
}

// calculate tracking error comparing commanded steps against as5600 angle readings
bool tmc_get_position_tracking_error(uint8_t addr, float *out_error_deg)
{
    if (addr >= TMC_NUM_MOTORS) return false;

    // motor step math: 200 full steps * microstepping
    float steps_per_motor_rev = 200.0f * (float)motors[addr].microsteps;
    if (steps_per_motor_rev <= 0.0f) steps_per_motor_rev = 200.0f;

    float motor_turns = (float)motors[addr].total_commanded_steps / steps_per_motor_rev;
    motors[addr].commanded_motor_deg = motor_turns * 360.0f;

    // get gear ratio and invert flag from as5600 module
    float gear_ratio = (addr < AS5600_NUM_ENCODERS && encoders[addr].gear_ratio > 0.0f)
                           ? encoders[addr].gear_ratio : 20.0f;
    bool invert = (addr < AS5600_NUM_ENCODERS) ? encoders[addr].invert_direction : true;
    float dir_sign = invert ? -1.0f : 1.0f;

    motors[addr].commanded_output_deg = dir_sign * (motors[addr].commanded_motor_deg / gear_ratio);

    // motor shaft error (as5600 is directly on the rear motor shaft)
    float actual_motor_deg = as5600_get_motor_angle_deg(addr);
    motors[addr].tracking_error_motor_deg = motors[addr].commanded_motor_deg - actual_motor_deg;

    // joint output shaft error
    float actual_output_deg = as5600_get_output_angle_deg(addr);
    motors[addr].tracking_error_deg = motors[addr].commanded_output_deg - actual_output_deg;

    if (out_error_deg != NULL) {
        *out_error_deg = motors[addr].tracking_error_deg;
    }
    return true;
}

// zero out commanded steps and encoder position
void tmc_reset_position(uint8_t addr)
{
    if (addr < TMC_NUM_MOTORS) {
        motors[addr].total_commanded_steps = 0;
        motors[addr].commanded_motor_deg = 0.0f;
        motors[addr].commanded_output_deg = 0.0f;
        motors[addr].tracking_error_deg = 0.0f;
        as5600_zero_output_position(addr);
    }
}

// dummy trajectory / kinematics hook for joint movement
void robot_arm_joint_move_dummy(uint8_t joint_id, float target_angle_deg, float speed_deg_s)
{
    if (joint_id >= TMC_NUM_MOTORS) return;

    float current_out_deg = as5600_get_output_angle_deg(joint_id);
    ESP_LOGI("ROBOT_DUMMY", "Joint %u commanded to %.2f deg (current=%.2f deg, err=%.2f deg) at %.2f deg/s",
             joint_id, target_angle_deg, current_out_deg,
             target_angle_deg - current_out_deg, speed_deg_s);

    // convert joint deg/s into motor step rate across 20:1 reduction
    float steps_per_motor_rev = 200.0f * (float)motors[joint_id].microsteps;
    float steps_per_out_deg = (steps_per_motor_rev * 20.0f) / 360.0f;
    int steps_per_sec = (int)(speed_deg_s * steps_per_out_deg);
    if (steps_per_sec < 100) steps_per_sec = 100;
    if (steps_per_sec > 12000) steps_per_sec = 12000;

    tmc_set_motor_speed_hz(joint_id, steps_per_sec);
}

// multi-joint trajectory stream callback
void robot_arm_stream_trajectory_dummy(const float joint_angles[5], const float joint_speeds[5])
{
    ESP_LOGI("ROBOT_DUMMY", "Streaming 5-DOF Trajectory: J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f, J4=%.1f",
             joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4]);

    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            robot_arm_joint_move_dummy(i, joint_angles[i], joint_speeds[i]);
        }
    }
}

// stallguard event handler: shuts down the motor that hit contact
void robot_arm_stallguard_event_callback(uint8_t joint_id, uint16_t sg_result)
{
    ESP_LOGW("ROBOT_SAFETY", ">>> STALLGUARD EVENT: Joint %u obstacle/contact detected (SG_RESULT=%u)!",
             joint_id, sg_result);
    motors[joint_id].target_speed_hz = 0;
    if (motors[joint_id].motion_mode == MOTION_MODE_UART_VACTUAL) {
        tmc_stop_vactual(joint_id);
    }
}

// sensorless homing across all detected motors
void robot_arm_sync_home_all_dummy(void)
{
    ESP_LOGI("ROBOT_DUMMY", "Triggering sensorless homing sequence for all detected joints...");
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            tmc_sensorless_homing(i, 2000, 64);
        }
    }
}
