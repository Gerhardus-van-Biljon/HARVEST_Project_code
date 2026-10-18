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
#define STEP_PULSE_US          5               // 5 us pulse width (matches working script)
#define POT_DEADBAND_LOW       40              // Deadband count for solid zero
#define MIN_RUN_SPEED_HZ       100             // 100 Hz starting step rate (matches working script)
#define MAX_SPEED_HZ           12000           // 12000 Hz max step rate (matches working script)

static volatile int target_speed_hz  = MIN_RUN_SPEED_HZ;
static volatile int current_speed_hz = 0;
static volatile bool timer_running   = false;
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

    if (adc_raw <= POT_DEADBAND_LOW) {
        return 0;
    }

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
// Step Pulse Timer Callback (Matching Verified Script)
// -----------------------------------------------------------------------------
static void step_timer_callback(void *arg)
{
    int speed = target_speed_hz;
    if (speed <= 0) {
        timer_running = false;
        current_speed_hz = 0;
        return;
    }

    // Simultaneously pulse all motor STEP pins (GPIO 9, 5, 7, 12)
    gpio_set_level(GPIO_NUM_9, 1);
    gpio_set_level(GPIO_NUM_5, 1);
    gpio_set_level(GPIO_NUM_7, 1);
    gpio_set_level(GPIO_NUM_12, 1);

    esp_rom_delay_us(STEP_PULSE_US);

    gpio_set_level(GPIO_NUM_9, 0);
    gpio_set_level(GPIO_NUM_5, 0);
    gpio_set_level(GPIO_NUM_7, 0);
    gpio_set_level(GPIO_NUM_12, 0);

    current_speed_hz = speed;
    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].current_speed_hz = speed;
        motors[i].target_speed_hz = speed;
        motors[i].total_commanded_steps++;
    }

    int64_t period_us = 1000000LL / speed;
    if (period_us < 50) {
        period_us = 50;
    }

    esp_timer_start_once(step_timer, period_us);
}

// -----------------------------------------------------------------------------
// Application Main
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "==================================================");
    ESP_LOGI(TAG_MAIN, "  ESP32-S3 TMC2209 STEPPER CONTROLLER STARTING    ");
    ESP_LOGI(TAG_MAIN, "==================================================");

    // 1. Initialize data structures & hardware GPIOs
    tmc_init_motor_structures();
    tmc_init_hardware_pins();

    // Explicitly configure all step/dir/en pins
    gpio_set_level(GPIO_NUM_4, 0);      // Active low enable
    gpio_set_level(GPIO_NUM_2, 1);      // Direction
    gpio_set_level(GPIO_NUM_6, 1);
    gpio_set_level(GPIO_NUM_8, 1);
    gpio_set_level(GPIO_NUM_13, 1);
    gpio_set_level(GPIO_NUM_9, 0);      // Step low
    gpio_set_level(GPIO_NUM_5, 0);
    gpio_set_level(GPIO_NUM_7, 0);
    gpio_set_level(GPIO_NUM_12, 0);

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

    // 6. Scan and configure drivers on UART (5 verified register writes)
    tmc_scan_and_configure_all();

    // 7. Create step timer (single-shot reschedule, matching verified script)
    esp_timer_create_args_t timer_args = {
        .callback = &step_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "step_timer",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &step_timer));

    int pot_raw = 0;
    int pot_speed = read_pot_mapped(&pot_raw);
    target_speed_hz = pot_speed;
    if (pot_speed > 0) {
        timer_running = true;
        ESP_ERROR_CHECK(esp_timer_start_once(step_timer, 1000000LL / pot_speed));
    }

    ESP_LOGI(TAG_MAIN, "Stepper engine active: pulses routed to GPIO 9, 5, 7, 12");
    ESP_LOGI(TAG_MAIN, "Entering real-time potentiometer demo loop...");

    uint32_t loop_counter = 0;

    while (1) {
        int raw = 0;
        int speed = read_pot_mapped(&raw);
        target_speed_hz = speed;

        // Wake up timer if resuming from stop
        if (speed > 0 && !timer_running) {
            timer_running = true;
            esp_timer_start_once(step_timer, 1000000LL / speed);
        }

        // Keep enable active LOW and direction set
        gpio_set_level(GPIO_NUM_4, 0);
        gpio_set_level(GPIO_NUM_2, 1);
        gpio_set_level(GPIO_NUM_6, 1);
        gpio_set_level(GPIO_NUM_8, 1);
        gpio_set_level(GPIO_NUM_13, 1);

        // Print telemetry report every ~500 ms (every 5 loop iterations)
        if ((loop_counter % 5) == 0) {
            tmc_print_all_motors_status();
            ESP_LOGI(TAG_MAIN, "[POT DEMO] ADC Raw: %4d | Commanded: %5d Hz | Actual: %5d Hz %s",
                     raw, speed, current_speed_hz, (speed == 0) ? "(STOPPED)" : "(RUNNING)");
        }

        loop_counter++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}