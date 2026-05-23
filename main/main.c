/*
 * CF-Drone firmware ported to ESP-IDF
 *
 * Main loop mirrors CF-Drone loop() structure:
 *   readIMU → step[dt] → readRC → estimate → battery → control → sendMotors → LED
 *
 * FreeRTOS task structure:
 *   drone_task  (Core 0, ~1kHz): IMU-driven fast loop
 *   aux_task    (Core 1, ~10Hz): battery, LED, logging
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// Component includes
#include "math_utils.h"
#include "imu.h"
#include "rc_sbus.h"
#include "motors.h"
#include "battery.h"
#include "led_ctrl.h"
#include "attitude_estimator.h"
#include "flight_control.h"
#include "drone_types.h"

// Networking / Web RC
#include "nvs_flash.h"
#include "wifi_ap.h"
#include "web_rc.h"
#include "tof_sensor.h"

static const char *TAG = "MAIN";

// ============================================================
// Global shared flight state
// ============================================================
static flight_state_t s_fs;

// ============================================================
// Time management (mirrors time.ino)
// ============================================================
static void step_time(void) {
    static int64_t prev_us = 0;
    int64_t now_us = esp_timer_get_time();

    if (prev_us == 0) {
        s_fs.dt = 0;
    } else {
        s_fs.dt = (now_us - prev_us) * 1e-6f;
        if (s_fs.dt < 0 || s_fs.dt > 1.0f) s_fs.dt = 0;
    }
    prev_us = now_us;
    s_fs.t  = now_us * 1e-6f;

    // Compute loop rate (1-second window)
    static float window_start = 0;
    static uint32_t rate_count = 0;
    rate_count++;
    if (s_fs.t - window_start >= 1.0f) {
        s_fs.loop_rate = (float)rate_count;
        window_start = s_fs.t;
        rate_count = 0;
    }
}

// ============================================================
// Fast loop task - Core 0
// Sequence mirrors CF-Drone loop():
//   readIMU → step → readRC → estimate → control → sendMotors
// ============================================================
static void drone_task(void *arg) {
    ESP_LOGI(TAG, "Drone task started on Core %d", xPortGetCoreID());

    imu_data_t  imu_data = {0};
    rc_input_t  rc_data  = {0};

    while (1) {
        // 1. Read IMU (blocking until data ready, ~1kHz)
        imu_read(&imu_data);

        // 2. Update time delta
        step_time();

        // 3. Calibrate gyro bias when stationary (landed)
        if (s_fs.landed) {
            imu_calibrate_gyro(&imu_data);
        }

        // 4. Read RC (non-blocking, returns last frame if no new)
        rc_sbus_read(&rc_data);
        if (!rc_data.valid) {
            web_rc_read(&rc_data);
        }

        // 5. Attitude estimation: gyro integration + acc correction
        attitude_estimator_update(&imu_data, &s_fs);

        // 6. Đọc độ cao từ TOF sensor (thread-safe, non-blocking)
        s_fs.tof_valid = tof_sensor_get_altitude(&s_fs.alt_measured_m);

        // 7. Flight control: interpret RC → PID → motor mix + safety
        fc_update(&s_fs, &rc_data);

        // 7. Write motor values to hardware
        float motor_vals[4];
        fc_get_motors(motor_vals);
        motors_set(motor_vals);
        motors_send();

        // vTaskDelay(1): ensures IDLE0 gets CPU time to reset the Task Watchdog.
        // SPI polling transfers are fast (~50µs for 14 bytes at 20MHz),
        // so this 1-tick delay (1ms) caps the loop at ~1kHz and prevents TWDT.
        vTaskDelay(1);
    }
}

// ============================================================
// Auxiliary loop task - Core 1
// Sequence: battery → LED → log
// ============================================================
static void aux_task(void *arg) {
    ESP_LOGI(TAG, "Aux task started on Core %d", xPortGetCoreID());

    static float last_log_t = 0;

    while (1) {
        // Battery voltage update (~2Hz)
        battery_update();
        s_fs.battery_voltage = battery_get_voltage();

        // LED status update
        led_ctrl_update(&s_fs);

        // Log telemetry to console at ~2Hz
        if (s_fs.t - last_log_t >= 0.5f) {
            last_log_t = s_fs.t;
            vec3_t euler = quat_to_euler(s_fs.attitude);
            ESP_LOGI(TAG,
                     "Mode=%-6s Armed=%d Rate=%.0fHz | "
                     "Roll=%.1f Pitch=%.1f Yaw=%.1f | "
                     "Thrust=%.2f Alt=%.3fm Batt=%.2fV",
                     fc_get_mode_name(s_fs.mode),
                     s_fs.armed,
                     s_fs.loop_rate,
                     euler.x * 180 / M_PI,
                     euler.y * 180 / M_PI,
                     euler.z * 180 / M_PI,
                     s_fs.thrust_target,
                     s_fs.alt_measured_m,
                     s_fs.battery_voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // ~20Hz
    }
}

// ============================================================
// App Main - Initialization
// Mirrors CF-Drone setup()
// ============================================================
void app_main(void) {
    ESP_LOGI(TAG, "=== CF-Drone ESP-IDF ===");
    ESP_LOGI(TAG, "Initializing...");

    // Initialize NVS (Required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Zero out flight state
    memset(&s_fs, 0, sizeof(s_fs));
    s_fs.attitude        = (quaternion_t){1, 0, 0, 0}; // identity
    s_fs.attitude_target = (quaternion_t){NAN, NAN, NAN, NAN}; // invalid
    s_fs.mode            = FLIGHT_MODE_STAB;
    s_fs.armed           = false;
    s_fs.alt_measured_m  = -1.0f;  // Chưa có dữ liệu TOF
    s_fs.alt_target_m    = 0.0f;
    s_fs.tof_valid       = false;

    // Initialize all hardware
    led_ctrl_init();
    led_ctrl_set(true); // LED ON during init

    motors_init();
    motors_stop();

    battery_init();

    imu_init();
    attitude_estimator_init();

    rc_sbus_init();
    fc_init();

    // TOF Sensor (I2C: SDA=GPIO16, SCL=GPIO17)
    if (!tof_sensor_init()) {
        ESP_LOGW(TAG, "TOF sensor init FAILED – ALTHOLD mode unavailable");
    }

    wifi_ap_init();
    web_rc_init();

    led_ctrl_set(false); // LED OFF after init
    ESP_LOGI(TAG, "Initialization complete!");
    ESP_LOGI(TAG, "ARM: Throttle LOW + Yaw RIGHT");
    ESP_LOGI(TAG, "DISARM: Throttle LOW + Yaw LEFT");

    // Launch FreeRTOS tasks
    xTaskCreatePinnedToCore(
        drone_task, "drone_task",
        8192, NULL, 10, NULL, 0);  // Core 0, high priority

    xTaskCreatePinnedToCore(
        tof_sensor_task, "tof_task",
        4096, NULL, 6, NULL, 1);   // Core 1, medium-high priority

    xTaskCreatePinnedToCore(
        aux_task, "aux_task",
        4096, NULL, 5, NULL, 1);   // Core 1, lower priority
}
