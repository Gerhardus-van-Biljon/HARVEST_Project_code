/**
 * @file main.c
 * @brief Multi-Motor Stepper Controller for ESP32-S3 with 4x TMC2209 Drivers.
 * 
 * Supports simultaneous control of 4 TMC2209 drivers on addresses 0, 1, 2, and 3.
 * Features:
 * - Simultaneous 4-motor pulse generation via high-resolution periodic timer
 * - Potentiometer analog speed regulation
 * - Direct UART velocity mode (VACTUAL)
 * - StallGuard4 load monitoring & sensorless stall detection
 * - CoolStep dynamic current reduction
 * - Microstepping resolution & chopper configuration over UART
 * - Hardware direction inversion & software enable/freewheeling
 * - Modular robotics trajectory and joint positioning hooks
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
#include "tmc2209.h"
#include "ota_update.h"

#define TAG_MAIN               "MAIN_APP"

// -----------------------------------------------------------------------------
// Pin & Peripheral Definitions
// -----------------------------------------------------------------------------
#define POT_PIN                GPIO_NUM_1      // ADC1_CH0 on ESP32-S3
#define UART_TX_PIN            GPIO_NUM_10     // ESP32 TX -> TMC2209 PDN_UART
#define UART_RX_PIN            GPIO_NUM_11     // ESP32 RX <- TMC2209 PDN_UART
#define UART_PORT              UART_NUM_1

// -----------------------------------------------------------------------------
// Speed & Stepping Parameters (Potentiometer Demo)
// -----------------------------------------------------------------------------
#define STEP_PULSE_US          2               // TMC2209 requires min 100ns pulse width
#define POT_DEADBAND_LOW       80              // ADC counts <= this threshold stop the motors
#define MIN_RUN_SPEED_HZ       150             // Minimum starting step rate in Hz
#define MAX_SPEED_HZ           8000            // Maximum demo step rate in Hz
#define TIMER_BASE_INTERVAL_US 50              // 50 us timer tick (20 kHz base rate)

static esp_timer_handle_t step_timer = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

// -----------------------------------------------------------------------------
// Potentiometer Analog Reading
// -----------------------------------------------------------------------------
static void adc_setup(void)
{
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
}

static int read_pot_mapped(int *out_raw)
{
    int adc_raw = 0;
    if (adc_handle != NULL) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw);
    }
    if (out_raw != NULL) {
        *out_raw = adc_raw;
    }

    // Low deadband: turning pot fully counter-clockwise stops all motors
    if (adc_raw <= POT_DEADBAND_LOW) {
        return 0;
    }

    // Map [POT_DEADBAND_LOW .. 4095] smoothly to [MIN_RUN_SPEED_HZ .. MAX_SPEED_HZ]
    int mapped = MIN_RUN_SPEED_HZ + ((adc_raw - POT_DEADBAND_LOW) * (MAX_SPEED_HZ - MIN_RUN_SPEED_HZ)) / (4095 - POT_DEADBAND_LOW);
    if (mapped > MAX_SPEED_HZ) mapped = MAX_SPEED_HZ;
    return mapped;
}

// -----------------------------------------------------------------------------
// UART Bus Initialization (Single-Wire / Half-Duplex for TMC2209)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Multi-Motor Step Pulse Timer Callback
// -----------------------------------------------------------------------------
static void step_timer_callback(void *arg)
{
    bool pulse[TMC_NUM_MOTORS] = {false};
    bool any_pulse = false;

    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        // Skip drivers that are not present, disabled, or controlled via UART VACTUAL
        if (!motors[i].present || !motors[i].enabled || motors[i].is_stalled) {
            continue;
        }
        if (motors[i].motion_mode == MOTION_MODE_UART_VACTUAL) {
            continue;
        }

        int speed = motors[i].target_speed_hz;
        if (speed <= 0) {
            motors[i].current_speed_hz = 0;
            continue;
        }

        motors[i].step_accum_us += TIMER_BASE_INTERVAL_US;
        int32_t step_interval_us = 1000000 / speed;
        if (step_interval_us < TIMER_BASE_INTERVAL_US) {
            step_interval_us = TIMER_BASE_INTERVAL_US;
        }

        if (motors[i].step_accum_us >= step_interval_us) {
            motors[i].step_accum_us -= step_interval_us;
            pulse[i] = true;
            any_pulse = true;
            gpio_set_level(motors[i].step_pin, 1);
            motors[i].current_speed_hz = speed;
            if (motors[i].dir_level) {
                motors[i].total_commanded_steps++;
            } else {
                motors[i].total_commanded_steps--;
            }
        }
    }

    if (any_pulse) {
        esp_rom_delay_us(STEP_PULSE_US);
        for (int i = 0; i < TMC_NUM_MOTORS; i++) {
            if (pulse[i]) {
                gpio_set_level(motors[i].step_pin, 0);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Application Main
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "==================================================");
    ESP_LOGI(TAG_MAIN, "  ESP32-S3 TMC2209 4-MOTOR CONTROLLER STARTING    ");
    ESP_LOGI(TAG_MAIN, "==================================================");

    // 1. Initialize data structures & hardware GPIOs
    tmc_init_motor_structures();
    tmc_init_hardware_pins();

    // 2. Initialize ADC for analog potentiometer speed control
    adc_setup();

    // 3. Initialize NVS Flash (required for Wi-Fi stack & persistent calibration)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    // 4. Initialize Wireless Over-The-Air (OTA) System (SoftAP + Web Server)
    ota_system_init(OTA_DEFAULT_STA_SSID, OTA_DEFAULT_STA_PASS);
    ota_confirm_app_valid();
    ota_print_network_info();

    // 5. Initialize half-duplex UART for Trinamic bus
    uart_setup();
    vTaskDelay(pdMS_TO_TICKS(150));      // Driver power stabilization time

    // 4. Scan and configure all 4 driver addresses (0, 1, 2, 3)
    tmc_scan_and_configure_all();

    // 5. Example setup: Configure StallGuard4 & CoolStep on active motors
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            // Set StallGuard sensitivity threshold (e.g., 64)
            tmc_stallguard_set_threshold(i, 64);
            // Enable CoolStep automatic current scaling
            tmc_coolstep_enable(i, 2, 5, 0, 0, false);
            // Set microsteps to 1/16 with 256 interpolation
            tmc_set_microsteps(i, 16, true);
        }
    }

    // 6. Start the periodic multi-motor pulse generation timer
    esp_timer_create_args_t timer_args = {
        .callback = &step_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tmc_step_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &step_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(step_timer, TIMER_BASE_INTERVAL_US));

    ESP_LOGI(TAG_MAIN, "Timer pulse engine running at %d us tick resolution", TIMER_BASE_INTERVAL_US);
    ESP_LOGI(TAG_MAIN, "Entering real-time potentiometer demo loop...");

    uint32_t loop_counter = 0;

    while (1) {
        // Read target speed and raw ADC value from analog potentiometer
        int pot_raw = 0;
        int pot_speed = read_pot_mapped(&pot_raw);

        // Update target speed and check StallGuard for all present motors
        for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
            if (motors[i].present) {
                // Keep motor enabled
                gpio_set_level(motors[i].en_pin, 0);

                // If not running in UART VACTUAL mode, update step frequency
                if (motors[i].motion_mode == MOTION_MODE_STEP_DIR) {
                    tmc_set_motor_speed_hz(i, pot_speed);
                }

                // Periodically check StallGuard4 status over UART (every 100 ms)
                if ((loop_counter % 2) == 0) {
                    bool stalled = false;
                    tmc_stallguard_check_stall(i, &stalled);
                }
            } else {
                // If a driver was missing at boot, attempt periodic retry probe
                if ((loop_counter % 20) == 0) {
                    if (tmc_probe_motor(i, false)) {
                        ESP_LOGI(TAG_MAIN, "Driver at address %u hot-plugged! Configuring...", i);
                        tmc_configure_motor(i);
                    }
                }
            }
        }

        // Print telemetry report every ~500 ms (every 10 loop iterations)
        if ((loop_counter % 10) == 0) {
            tmc_print_all_motors_status();
            ESP_LOGI(TAG_MAIN, "[POT DEMO] ADC Raw: %4d | Commanded Speed: %5d Hz %s",
                     pot_raw, pot_speed, (pot_speed == 0) ? "(STOPPED)" : "(RUNNING)");
        }

        loop_counter++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}