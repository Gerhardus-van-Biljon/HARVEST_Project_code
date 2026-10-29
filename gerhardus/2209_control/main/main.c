/*
 * main app: 5-motor arm controller running on esp32-s3.
 * drives 5x tmc2209 modules, reads 5x as5600 encoders over a tca9548a multiplexer,
 * handles speed pot input, wifi ota updates, and runs the step pulse timer.
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
#include "as5600.h"
#include "ota_update.h"

#define TAG_MAIN               "MAIN_APP"

// pot input pin on esp32-s3 adc1
#define POT_PIN                GPIO_NUM_4      // ADC1_CH3

// pulse timing and step rate bounds
#define STEP_PULSE_US          2               // tmc2209 only needs 100ns min, 2us is plenty safe
#define MIN_SPEED_HZ           100             // slowest step frequency
#define MAX_SPEED_HZ           12000           // top step frequency
#define TIMER_BASE_INTERVAL_US 50              // 50us tick rate = 20 khz base timer

static esp_timer_handle_t step_timer = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

// configure adc1 channel 0 for reading our analog speed pot
static void adc_setup(void)
{
    // pull-down prevents motor taking off if pot pin floats
    gpio_set_pull_mode(POT_PIN, GPIO_PULLDOWN_ONLY);

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &channel_cfg));
}

// read adc and convert to step frequency with deadband at bottom
static int read_pot_mapped(void)
{
    int adc_raw = 0;
    if (adc_handle != NULL) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &adc_raw);
    }

    // bottom 5% deadband so motor is dead stopped when turned all the way down
    if (adc_raw < 150) {
        return 0;
    }

    // map remainder (150..4095) across 200 hz to max speed
    int mapped = ((adc_raw - 150) * (MAX_SPEED_HZ - 200)) / (4095 - 150);
    mapped += 200;
    if (mapped > MAX_SPEED_HZ) mapped = MAX_SPEED_HZ;
    return mapped;
}

// high-frequency step pulse timer callback (fires every 50us)
static void step_timer_callback(void *arg)
{
    bool pulse[TMC_NUM_MOTORS] = {false};
    bool any_pulse = false;

    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        // skip disabled drivers or motors running via internal uart vactual mode
        if (!motors[i].present || !motors[i].enabled || motors[i].is_stalled) {
            motors[i].current_speed_hz = 0;
            continue;
        }
        if (motors[i].motion_mode == MOTION_MODE_UART_VACTUAL) {
            continue;
        }

        int target = motors[i].target_speed_hz;
        if (target <= 0) {
            motors[i].current_speed_hz = 0;
            continue;
        }

        // smooth speed ramping: +2 hz per 50us (~40k hz/s^2 acceleration)
        if (motors[i].current_speed_hz < target) {
            motors[i].current_speed_hz += 2;
            if (motors[i].current_speed_hz > target) {
                motors[i].current_speed_hz = target;
            }
        } else if (motors[i].current_speed_hz > target) {
            motors[i].current_speed_hz -= 4;
            if (motors[i].current_speed_hz < target) {
                motors[i].current_speed_hz = target;
            }
        }

        int active_speed = motors[i].current_speed_hz;
        if (active_speed <= 0) {
            continue;
        }

        motors[i].step_accum_us += TIMER_BASE_INTERVAL_US;
        int32_t step_interval_us = 1000000 / active_speed;
        if (step_interval_us < TIMER_BASE_INTERVAL_US) {
            step_interval_us = TIMER_BASE_INTERVAL_US;
        }

        if (motors[i].step_accum_us >= step_interval_us) {
            motors[i].step_accum_us -= step_interval_us;
            pulse[i] = true;
            any_pulse = true;
            gpio_set_level(motors[i].step_pin, 1);
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

// main entry point
void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "==================================================");
    ESP_LOGI(TAG_MAIN, "  ESP32-S3 TMC2209 5-MOTOR CONTROLLER STARTING    ");
    ESP_LOGI(TAG_MAIN, "==================================================");

    // 1. set up motor structs and configure gpio output pins
    tmc_init_motor_structures();
    tmc_init_hardware_pins();

    // 2. set up adc for the speed pot
    adc_setup();

    // 3. nvs flash needed for wifi credentials and ota rollbacks
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    // 4. bring up wireless ota server (access point mode + web upload portal)
    ota_system_init(OTA_DEFAULT_STA_SSID, OTA_DEFAULT_STA_PASS);
    ota_confirm_app_valid();
    ota_print_network_info();

    // 5. fire up both half-duplex uart buses (uart1 for m0..m3, uart2 for m4)
    ESP_ERROR_CHECK(tmc_uart_bus_init());
    vTaskDelay(pdMS_TO_TICKS(150));      // let drivers power up and stabilize

    // 6. scan for drivers on both buses and configure registers
    tmc_scan_and_configure_all();

    // 7. start i2c bus and detect as5600 encoders on mux channels 0..4
    as5600_i2c_bus_init(AS5600_I2C_DEFAULT_SDA, AS5600_I2C_DEFAULT_SCL, AS5600_I2C_FREQ_HZ);
    as5600_init_all();

    // 8. set up currents, stallguard, coolstep and microstepping for active motors
    for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
        if (motors[i].present) {
            // motor 0 is 2.0A 60mm base motor, motors 1..4 are 17HS3401S 1.3A peak
            if (i == 0) {
                tmc_set_current(i, 25, 12, 1);
            } else {
                tmc_set_current(i, 16, 8, 1);
            }
            tmc_stallguard_set_threshold(i, 64);
            tmc_coolstep_enable(i, 2, 5, 0, 0, false);
            tmc_set_microsteps(i, 16, true);
        }
    }

    // 9. start the periodic step generation timer
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
    ESP_LOGI(TAG_MAIN, "Entering real-time control loop...");

    uint32_t loop_counter = 0;

    while (1) {
        // read all 5 magnetic encoders over the multiplexer
        as5600_update_all();

        // read speed setting from potentiometer
        int pot_speed = read_pot_mapped();

        // update motors and check stall status
        for (uint8_t i = 0; i < TMC_NUM_MOTORS; i++) {
            if (motors[i].present) {
                // keep drivers active
                gpio_set_level(motors[i].en_pin, 0);

                // apply pot speed if turned up and in step/dir mode
                if (pot_speed > 0 && motors[i].motion_mode == MOTION_MODE_STEP_DIR) {
                    tmc_set_motor_speed_hz(i, pot_speed);
                }

                // check tracking error against encoder feedback
                float tracking_err_deg = 0.0f;
                tmc_get_position_tracking_error(i, &tracking_err_deg);

                // check stallguard every 100ms
                if ((loop_counter % 2) == 0) {
                    bool stalled = false;
                    tmc_stallguard_check_stall(i, &stalled);
                }
            } else {
                // if a driver was missing at startup, probe again every ~1s
                if ((loop_counter % 20) == 0) {
                    if (tmc_probe_motor(i, false)) {
                        ESP_LOGI(TAG_MAIN, "Driver at address %u hot-plugged! Configuring...", i);
                        tmc_configure_motor(i);
                    }
                }
            }
        }

        // telemetry readout every 500ms
        if ((loop_counter % 10) == 0) {
            tmc_print_all_motors_status();
            as5600_print_telemetry();
        }

        // dummy trajectory demo hook every 2.5s
        if ((loop_counter % 50) == 0 && loop_counter > 0) {
            float dummy_angles[5] = {15.0f, 30.0f, 45.0f, 60.0f, 75.0f};
            float dummy_speeds[5] = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f};
            robot_arm_stream_trajectory_dummy(dummy_angles, dummy_speeds);
        }

        loop_counter++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
