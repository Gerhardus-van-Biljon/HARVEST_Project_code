/**
 * @file main.c
 * @brief Stepper Controller for ESP32-S3 with TMC2209 Driver.
 * 
 * Potentiometer Demo with Thermal Protection & Smooth Acceleration Ramping:
 * - 4 microsteps (CHOPCONF 0x16000054u) with 256 interpolation
 * - Safe thermal current scaling: IRUN=20 (~0.95A RMS), IHOLD=6 (~0.28A RMS)
 *   to eliminate extreme motor overheating while providing full gearbox torque
 * - SpreadCycle chopper mode (GCONF 0x000000C4u)
 * - Smooth 2000 Hz/s acceleration ramping to prevent motor stalling / pole slipping
 * - Low-end deadband (ADC < 120 -> 0 Hz / standstill idle)
 * - Single-cycle simultaneous step pulse generation to GPIO 9 and GPIO 12
 * - Integrated Wireless OTA web portal (SoftAP + Station)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "ota_update.h"
#include "soc/gpio_reg.h"

// ---------------------------------------------------------------- pins
// Primary motor (Socket 0 / Joint 0)
#define STEP_PIN          GPIO_NUM_9      // Motor 0 STEP pin
#define DIR_PIN           GPIO_NUM_2      // Motor 0 DIR pin
#define EN_PIN            GPIO_NUM_4      // Shared motor EN pin (Active LOW)

// Secondary motor pins (Socket 1, 2, 3)
#define MOTOR1_STEP_PIN   GPIO_NUM_5
#define MOTOR1_DIR_PIN    GPIO_NUM_6
#define MOTOR2_STEP_PIN   GPIO_NUM_7
#define MOTOR2_DIR_PIN    GPIO_NUM_8
#define MOTOR3_STEP_PIN   GPIO_NUM_12     // Motor 3 STEP pin (Address 3)
#define MOTOR3_DIR_PIN    GPIO_NUM_13     // Motor 3 DIR pin

#define MOTOR_STEP_PINS_MASK ((1ULL << STEP_PIN)        | \
                              (1ULL << MOTOR1_STEP_PIN) | \
                              (1ULL << MOTOR2_STEP_PIN) | \
                              (1ULL << MOTOR3_STEP_PIN))

#define POT_PIN           GPIO_NUM_1      // ADC1_CH0 on ESP32-S3
#define UART_TX_PIN       GPIO_NUM_10     // ESP32 TX -> driver RX
#define UART_RX_PIN       GPIO_NUM_11     // ESP32 RX <- driver TX
#define UART_PORT         UART_NUM_1

// ------------------------------------------------------------ stepping
#define STEP_PULSE_US         4       // 4 us pulse width
#define POT_DEADBAND_ADC      120     // ADC counts <= this threshold STOP the motor
#define MIN_SPEED_HZ          150     // Starting step frequency (smooth slow rotation)
#define MAX_SPEED_HZ          5000    // Max step frequency (clean ~15 RPM gearbox output)
#define RAMP_STEP_HZ          40      // 40 Hz step change per 20ms tick (~2000 Hz/s acceleration)

// ------------------------------------------------------- TMC2209 setup
// Cool, high-torque current scaling:
// IRUN=20 (~0.95A RMS) prevents motor overheating while easily turning the 20:1 gearbox
// IHOLD=6 (~0.28A RMS) drops standby power by 92% to keep motor cool at standstill
#define TMC_IRUN              20      // 0..31, run current scale (rated for NEMA 17)
#define TMC_IHOLD             6       // 0..31, standstill current scale
#define TMC_IHOLDDELAY        1

// Register addresses
#define REG_GCONF         0x00
#define REG_IFCNT         0x02
#define REG_IHOLD_IRUN    0x10
#define REG_TPOWERDOWN    0x11
#define REG_DRV_STATUS    0x6F
#define REG_CHOPCONF      0x6C
#define REG_PWMCONF       0x70

static const char *TAG = "tmc2209_main";

static volatile int  target_speed_hz  = 0;
static volatile int  current_speed_hz = 0;
static volatile bool timer_running    = false;
static esp_timer_handle_t step_timer  = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

static uint8_t tmc_addr  = 0;      // discovered at runtime (MS1/MS2 select 0..3)
static bool    tmc_ready = false;  // true once driver found and configured

// ------------------------------------------------------------------ CRC
// Computes CRC8 over datagram[0 .. length-2] and stores it in datagram[length-1]
static void tmc_calc_crc(uint8_t *datagram, uint8_t datagram_length)
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

// ------------------------------------------------------- register access
static void tmc_write_reg(uint8_t addr, uint8_t reg, uint32_t value)
{
    uint8_t datagram[8] = {0};

    datagram[0] = 0x05;
    datagram[1] = addr;
    datagram[2] = (uint8_t)(reg | 0x80u);
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

// Returns true only if a reply with correct sync, address (0xFF), register and CRC was received.
static bool tmc_read_reg_ex(uint8_t addr, uint8_t reg, uint32_t *out, bool verbose)
{
    uint8_t req[4] = {0x05, addr, reg, 0};
    tmc_calc_crc(req, sizeof(req));

    uint8_t buf[16];
    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, (const char *)req, sizeof(req));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(10));

    int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));

    if (verbose) {
        printf("Scanning Addr %u | TX: %02X %02X %02X %02X | RX (%d bytes):",
               addr, req[0], req[1], req[2], req[3], len);
        for (int i = 0; i < len; i++) printf(" %02X", buf[i]);
    }

    for (int i = 0; i + 8 <= len; i++) {
        if (buf[i] == 0x05 && buf[i + 1] == 0xFF && buf[i + 2] == reg) {
            uint8_t crc_buf[8];
            memcpy(crc_buf, &buf[i], 8);
            tmc_calc_crc(crc_buf, 8);
            if (crc_buf[7] != buf[i + 7]) {
                if (verbose) printf(" | bad CRC\n");
                return false;
            }
            *out = ((uint32_t)buf[i + 3] << 24) |
                   ((uint32_t)buf[i + 4] << 16) |
                   ((uint32_t)buf[i + 5] << 8)  |
                    (uint32_t)buf[i + 6];
            if (verbose) printf(" | BINGO! Valid reply on address %u\n", addr);
            return true;
        }
    }
    if (verbose) printf(" | no valid reply\n");
    return false;
}

static bool tmc_read_reg(uint8_t addr, uint8_t reg, uint32_t *out)
{
    return tmc_read_reg_ex(addr, reg, out, false);
}

// Probe addresses 0..3 by reading IFCNT; sets tmc_addr on success.
static bool tmc_find_driver(void)
{
    for (uint8_t addr = 0; addr < 4; addr++) {
        uint32_t v;
        if (tmc_read_reg_ex(addr, REG_IFCNT, &v, true)) {
            tmc_addr = addr;
            ESP_LOGI(TAG, "TMC2209 found at address %u (IFCNT=%u)", addr, (unsigned)v);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

// ------------------------------------------------------------ driver init
static void uart_setup(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 256, 0, NULL, 0));
}

static bool tmc2209_configure(void)
{
    if (!tmc_find_driver()) {
        return false;
    }

    uint32_t ifcnt_before = 0, ifcnt_after = 0;
    bool have_before = tmc_read_reg(tmc_addr, REG_IFCNT, &ifcnt_before);

    // GCONF: bit2 en_spreadCycle, bit6 pdn_disable, bit7 mstep_reg_select
    tmc_write_reg(tmc_addr, REG_GCONF, 0x000000C4u);

    // IHOLD_IRUN: IHOLD[4:0], IRUN[12:8], IHOLDDELAY[19:16]
    uint32_t ihold_irun = ((uint32_t)TMC_IHOLDDELAY << 16) |
                          ((uint32_t)TMC_IRUN << 8) |
                           (uint32_t)TMC_IHOLD;
    tmc_write_reg(tmc_addr, REG_IHOLD_IRUN, ihold_irun);

    tmc_write_reg(tmc_addr, REG_TPOWERDOWN, 0x00000004u);

    // CHOPCONF: TOFF=4, HSTRT=5, MRES=6 (4 microsteps), intpol=1
    tmc_write_reg(tmc_addr, REG_CHOPCONF, 0x16000054u);

    // PWMCONF: documented reset default (stealthChop autoscale on)
    tmc_write_reg(tmc_addr, REG_PWMCONF, 0xC10D0024u);

    // Verify the driver actually accepted our 5 writes
    bool have_after = tmc_read_reg(tmc_addr, REG_IFCNT, &ifcnt_after);
    if (have_before && have_after) {
        uint8_t delta = (uint8_t)(ifcnt_after - ifcnt_before);
        if (delta == 5) {
            ESP_LOGI(TAG, "TMC2209 configured OK (IFCNT %u -> %u, 5 writes accepted)",
                     (unsigned)ifcnt_before, (unsigned)ifcnt_after);
        } else {
            ESP_LOGW(TAG, "IFCNT changed by %u, expected 5 - some writes were rejected",
                     (unsigned)delta);
        }
    } else {
        ESP_LOGW(TAG, "Could not read IFCNT to verify writes");
    }
    return true;
}

// -------------------------------------------------------------------- pot
static int read_pot_mapped(int *raw_out)
{
    int adc_raw = 0;
    if (adc_handle != NULL) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw);
    }
    if (raw_out) *raw_out = adc_raw;

    // Bottom deadband: Stop motor completely when knob is turned down
    if (adc_raw < POT_DEADBAND_ADC) {
        return 0;
    }

    // Scale smoothly from MIN_SPEED_HZ to MAX_SPEED_HZ
    int span_adc = 4095 - POT_DEADBAND_ADC;
    int rel_adc = adc_raw - POT_DEADBAND_ADC;
    int mapped = MIN_SPEED_HZ + (rel_adc * (MAX_SPEED_HZ - MIN_SPEED_HZ)) / span_adc;

    if (mapped < MIN_SPEED_HZ) mapped = MIN_SPEED_HZ;
    if (mapped > MAX_SPEED_HZ) mapped = MAX_SPEED_HZ;
    return mapped;
}

// ------------------------------------------------------------ status print
static void tmc2209_print_status_debug(int raw_adc)
{
    uint32_t status = 0;
    if (!tmc_read_reg(tmc_addr, REG_DRV_STATUS, &status)) {
        printf("[POT DEMO] DRV_STATUS read failed (addr %u)\n", tmc_addr);
        return;
    }

    uint32_t otpw = (status >> 0) & 1u;
    uint32_t ot   = (status >> 1) & 1u;
    uint32_t s2ga = (status >> 2) & 1u;
    uint32_t s2gb = (status >> 3) & 1u;
    uint32_t ola  = (status >> 6) & 1u;
    uint32_t olb  = (status >> 7) & 1u;
    uint8_t  cs   = (status >> 16) & 0x1Fu;
    uint32_t stst = (status >> 31) & 1u;

    printf("[POT DEMO] ADC: %4d | Target: %4d Hz | Actual: %4d Hz | CS: %2u | Driver Addr %u | Flags:",
           raw_adc, target_speed_hz, current_speed_hz, (unsigned)cs, tmc_addr);

    if (otpw) printf(" OTPW(TEMP_WARN)");
    if (ot)   printf(" OT(OVERTEMP_SHUTDOWN)");
    if (s2ga) printf(" S2GA");
    if (s2gb) printf(" S2GB");
    if (ola)  printf(" OLA");
    if (olb)  printf(" OLB");
    if (stst) printf(" STANDSTILL");
    if (!(otpw | ot | s2ga | s2gb | ola | olb | stst)) printf(" OK_RUNNING");

    printf("\n");
    fflush(stdout);
}

// ----------------------------------------------------------- step timer
static void step_timer_callback(void *arg)
{
    int speed = current_speed_hz;
    if (speed <= 0) {
        timer_running = false;
        return;
    }

    // High-speed simultaneous pulse to all STEP pins (GPIO 9, 5, 7, 12)
    REG_WRITE(GPIO_OUT_W1TS_REG, (uint32_t)MOTOR_STEP_PINS_MASK);
    esp_rom_delay_us(STEP_PULSE_US);
    REG_WRITE(GPIO_OUT_W1TC_REG, (uint32_t)MOTOR_STEP_PINS_MASK);

    int64_t period_us = 1000000LL / speed;
    if (period_us < 80) {
        period_us = 80;
    }

    esp_err_t err = esp_timer_start_once(step_timer, period_us);
    if (err != ESP_OK) {
        timer_running = false;
    }
}

// ------------------------------------------------------------------ main
void app_main(void)
{
    // Configure all motor STEP, DIR, and EN pins as outputs
    uint64_t pin_mask = MOTOR_STEP_PINS_MASK |
                        (1ULL << DIR_PIN)        | (1ULL << EN_PIN) |
                        (1ULL << MOTOR1_DIR_PIN) |
                        (1ULL << MOTOR2_DIR_PIN) |
                        (1ULL << MOTOR3_DIR_PIN);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Initialize initial pin states
    gpio_set_level(EN_PIN, 0);           // Active LOW: all drivers enabled
    gpio_set_level(DIR_PIN, 1);          // Forward direction
    gpio_set_level(MOTOR1_DIR_PIN, 1);
    gpio_set_level(MOTOR2_DIR_PIN, 1);
    gpio_set_level(MOTOR3_DIR_PIN, 1);
    REG_WRITE(GPIO_OUT_W1TC_REG, (uint32_t)MOTOR_STEP_PINS_MASK); // STEP low

    // Initialize ADC for analog potentiometer speed control
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &channel_cfg));

    // Initialize half-duplex UART for Trinamic bus
    uart_setup();
    vTaskDelay(pdMS_TO_TICKS(100));      // Driver power stabilization time

    // Discover driver and configure registers
    tmc_ready = tmc2209_configure();
    if (!tmc_ready) {
        ESP_LOGW(TAG, "No TMC2209 reply on addresses 0-3. Check VIO, GND, TX/RX crossing, MS1/MS2. Will retry.");
    }

    // Initialize NVS Flash & Wireless OTA Web Portal
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    ota_system_init(OTA_DEFAULT_STA_SSID, OTA_DEFAULT_STA_PASS);
    ota_confirm_app_valid();
    ota_print_network_info();

    // Create single-shot step timer
    esp_timer_create_args_t timer_args = {
        .callback = &step_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "step_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &step_timer));

    ESP_LOGI(TAG, "ESP32-S3 port initialized; stepper driver and ADC ready (pulses on GPIO 9, 5, 7, 12)");
    ESP_LOGI(TAG, "Thermal protection active: IRUN=%d, IHOLD=%d, smooth acceleration ramping enabled",
             TMC_IRUN, TMC_IHOLD);

    uint32_t loop_counter = 0;

    while (1) {
        int raw_adc = 0;
        int target = read_pot_mapped(&raw_adc);
        target_speed_hz = target;

        // Smooth acceleration / deceleration ramping (slew rate)
        if (current_speed_hz < target) {
            current_speed_hz += RAMP_STEP_HZ;
            if (current_speed_hz > target) {
                current_speed_hz = target;
            }
        } else if (current_speed_hz > target) {
            current_speed_hz -= RAMP_STEP_HZ;
            if (current_speed_hz < target) {
                current_speed_hz = target;
            }
        }

        // If motor should be moving and timer is not currently running, wake it up
        if (current_speed_hz > 0 && !timer_running) {
            timer_running = true;
            int64_t period_us = 1000000LL / current_speed_hz;
            if (period_us < 80) period_us = 80;
            esp_timer_start_once(step_timer, period_us);
        }

        // Maintain direction and driver enable
        gpio_set_level(DIR_PIN, 1);
        gpio_set_level(MOTOR1_DIR_PIN, 1);
        gpio_set_level(MOTOR2_DIR_PIN, 1);
        gpio_set_level(MOTOR3_DIR_PIN, 1);
        gpio_set_level(EN_PIN, 0);

        if (!tmc_ready) {
            if ((loop_counter % 25) == 0) {
                tmc_ready = tmc2209_configure();   // keep retrying until driver answers
            }
        } else {
            if ((loop_counter % 25) == 0) {       // Telemetry printed every ~500 ms
                tmc2209_print_status_debug(raw_adc);
            }
        }

        loop_counter++;
        vTaskDelay(pdMS_TO_TICKS(20));             // 20ms update rate for smooth ramping
    }
}