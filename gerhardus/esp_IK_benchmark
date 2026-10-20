/* ESP32-S3 Feasibility Benchmark
   Simulates math load of CCD and Jacobian Transpose methods
   for inverse kinematics calculations.
*/
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "benchmark";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to
   blink, or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void) {
  /* If the addressable LED is enabled */
  if (s_led_state) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
  } else {
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
  }
}

static void configure_led(void) {
  ESP_LOGI(TAG, "Example configured to blink addressable LED!");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = BLINK_GPIO,
      .max_leds = 1, // at least one LED on board
  };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };
  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
  led_strip_spi_config_t spi_config = {
      .spi_bus = SPI2_HOST,
      .flags.with_dma = true,
  };
  ESP_ERROR_CHECK(
      led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void) {
  /* Set the GPIO level according to the state (LOW or HIGH)*/
  gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void) {
  ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
  gpio_reset_pin(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

// Fast inline PRNG (Xorshift32) to generate pseudo-random float/double values
// in [0.0, 1.0]
static inline uint32_t xorshift32(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static inline float rand_float(uint32_t *state) {
  return (float)xorshift32(state) / (float)4294967295U;
}

static inline double rand_double(uint32_t *state) {
  return (double)xorshift32(state) / (double)4294967295U;
}

// CCD Step - Float
void simulate_ccd_step_float(float current_joints[6], float target_pos[3],
                             float target_rot[9], uint32_t *prng_state) {
  for (int i = 5; i >= 0; i--) {
    float v1[3];
    float v2[3];
    v1[0] = rand_float(prng_state);
    v1[1] = rand_float(prng_state);
    v1[2] = rand_float(prng_state);

    v2[0] = rand_float(prng_state);
    v2[1] = rand_float(prng_state);
    v2[2] = rand_float(prng_state);

    float angle_delta = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) * 0.01f;
    current_joints[i] += angle_delta;
  }
}

// CCD Step - Double
void simulate_ccd_step_double(double current_joints[6], double target_pos[3],
                              double target_rot[9], uint32_t *prng_state) {
  for (int i = 5; i >= 0; i--) {
    double v1[3];
    double v2[3];
    v1[0] = rand_double(prng_state);
    v1[1] = rand_double(prng_state);
    v1[2] = rand_double(prng_state);

    v2[0] = rand_double(prng_state);
    v2[1] = rand_double(prng_state);
    v2[2] = rand_double(prng_state);

    double angle_delta = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) * 0.01;
    current_joints[i] += angle_delta;
  }
}

// Jacobian Transpose Step - Float
void simulate_jacobian_transpose_step_float(float current_joints[6],
                                            float target_pos[3],
                                            float target_rot[9],
                                            uint32_t *prng_state) {
  float J[6][6];
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      J[r][c] = rand_float(prng_state);
    }
  }

  float error[6];
  for (int i = 0; i < 6; i++) {
    error[i] = rand_float(prng_state);
  }

  float delta_theta[6] = {0};
  for (int i = 0; i < 6; i++) {
    float sum = 0.0f;
    for (int j = 0; j < 6; j++) {
      sum += J[j][i] * error[j];
    }
    delta_theta[i] = sum * 0.01f;
  }

  for (int i = 0; i < 6; i++) {
    current_joints[i] += delta_theta[i];
  }
}

// Jacobian Transpose Step - Double
void simulate_jacobian_transpose_step_double(double current_joints[6],
                                             double target_pos[3],
                                             double target_rot[9],
                                             uint32_t *prng_state) {
  double J[6][6];
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      J[r][c] = rand_double(prng_state);
    }
  }

  double error[6];
  for (int i = 0; i < 6; i++) {
    error[i] = rand_double(prng_state);
  }

  double delta_theta[6] = {0};
  for (int i = 0; i < 6; i++) {
    double sum = 0.0;
    for (int j = 0; j < 6; j++) {
      sum += J[j][i] * error[j];
    }
    delta_theta[i] = sum * 0.01;
  }

  for (int i = 0; i < 6; i++) {
    current_joints[i] += delta_theta[i];
  }
}

void app_main(void) {
  /* Configure the peripheral according to the LED type */
  configure_led();

  ESP_LOGI(TAG, "Waiting 3 seconds for monitor to connect...");
  vTaskDelay(pdMS_TO_TICKS(3000));

  const int iterations = 100000;
  const int block_size = 1000;
  const int num_blocks = iterations / block_size;

  ESP_LOGI(TAG, "Running ESP32-S3 IK Benchmark (%d iterations)", iterations);

  // 1. CCD Float
  float ccd_joints_f[6] = {0.0f};
  float target_p_f[3] = {0.2f, 0.1f, 0.3f};
  float target_r_f[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  uint32_t seed = 123456789U;

  int64_t ccd_time_f_total = 0;
  for (int b = 0; b < num_blocks; b++) {
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < block_size; i++) {
      simulate_ccd_step_float(ccd_joints_f, target_p_f, target_r_f, &seed);
    }
    ccd_time_f_total += (esp_timer_get_time() - start);
    vTaskDelay(1); // Force block for 1 tick to let IDLE task reset watchdog
  }

  // 2. CCD Double
  double ccd_joints_d[6] = {0.0};
  double target_p_d[3] = {0.2, 0.1, 0.3};
  double target_r_d[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  seed = 123456789U; // Reset seed to compare exactly

  int64_t ccd_time_d_total = 0;
  for (int b = 0; b < num_blocks; b++) {
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < block_size; i++) {
      simulate_ccd_step_double(ccd_joints_d, target_p_d, target_r_d, &seed);
    }
    ccd_time_d_total += (esp_timer_get_time() - start);
    vTaskDelay(1); // Force block for 1 tick to let IDLE task reset watchdog
  }

  // 3. Jacobian Transpose Float
  float jac_joints_f[6] = {0.0f};
  seed = 123456789U; // Reset seed

  int64_t jac_time_f_total = 0;
  for (int b = 0; b < num_blocks; b++) {
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < block_size; i++) {
      simulate_jacobian_transpose_step_float(jac_joints_f, target_p_f,
                                             target_r_f, &seed);
    }
    jac_time_f_total += (esp_timer_get_time() - start);
    vTaskDelay(1); // Force block for 1 tick to let IDLE task reset watchdog
  }

  // 4. Jacobian Transpose Double
  double jac_joints_d[6] = {0.0};
  seed = 123456789U; // Reset seed

  int64_t jac_time_d_total = 0;
  for (int b = 0; b < num_blocks; b++) {
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < block_size; i++) {
      simulate_jacobian_transpose_step_double(jac_joints_d, target_p_d,
                                              target_r_d, &seed);
    }
    jac_time_d_total += (esp_timer_get_time() - start);
    vTaskDelay(1); // Force block for 1 tick to let IDLE task reset watchdog
  }

  // Print Results Table
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "| Benchmark Name              | Precision    | Total Time "
                "(ms)  | Time/Iter (us)   |");

  double ccd_f_ms = (double)ccd_time_f_total / 1000.0;
  double ccd_f_us_per_iter = (double)ccd_time_f_total / (double)iterations;
  ESP_LOGI(
      TAG,
      "| CCD Simulation              | float (FPU)  | %12.2f     | %16.3f |",
      ccd_f_ms, ccd_f_us_per_iter);

  double ccd_d_ms = (double)ccd_time_d_total / 1000.0;
  double ccd_d_us_per_iter = (double)ccd_time_d_total / (double)iterations;
  ESP_LOGI(
      TAG,
      "| CCD Simulation              | double (EMU) | %12.2f     | %16.3f |",
      ccd_d_ms, ccd_d_us_per_iter);

  double jac_f_ms = (double)jac_time_f_total / 1000.0;
  double jac_f_us_per_iter = (double)jac_time_f_total / (double)iterations;
  ESP_LOGI(
      TAG,
      "| Jacobian Transpose          | float (FPU)  | %12.2f     | %16.3f |",
      jac_f_ms, jac_f_us_per_iter);

  double jac_d_ms = (double)jac_time_d_total / 1000.0;
  double jac_d_us_per_iter = (double)jac_time_d_total / (double)iterations;
  ESP_LOGI(
      TAG,
      "| Jacobian Transpose          | double (EMU) | %12.2f     | %16.3f |",
      jac_d_ms, jac_d_us_per_iter);

  // Print final values to ensure compiler optimization does not discard the
  // calculations
  ESP_LOGI(TAG, "\nFinal Joint Values to prevent compiler optimization:");
  ESP_LOGI(TAG, "CCD Float:    [%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f]",
           ccd_joints_f[0], ccd_joints_f[1], ccd_joints_f[2], ccd_joints_f[3],
           ccd_joints_f[4], ccd_joints_f[5]);
  ESP_LOGI(TAG, "CCD Double:   [%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f]",
           ccd_joints_d[0], ccd_joints_d[1], ccd_joints_d[2], ccd_joints_d[3],
           ccd_joints_d[4], ccd_joints_d[5]);
  ESP_LOGI(TAG, "Jac Trans F:  [%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f]",
           jac_joints_f[0], jac_joints_f[1], jac_joints_f[2], jac_joints_f[3],
           jac_joints_f[4], jac_joints_f[5]);
  ESP_LOGI(TAG, "Jac Trans D:  [%7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f]",
           jac_joints_d[0], jac_joints_d[1], jac_joints_d[2], jac_joints_d[3],
           jac_joints_d[4], jac_joints_d[5]);

  ESP_LOGI(TAG, "\nBenchmark Complete! Entering LED Blink loop...");

  while (1) {
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    blink_led();
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
  }
}
