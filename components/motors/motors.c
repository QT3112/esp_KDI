#include "motors.h"
#include "drone_config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MOTORS";

// Motor GPIO pins: RL, RR, FR, FL
static const int MOTOR_PINS[4] = {
    BOARD_MOTOR_PIN_RL,
    BOARD_MOTOR_PIN_RR,
    BOARD_MOTOR_PIN_FR,
    BOARD_MOTOR_PIN_FL
};

static float s_motors[4] = {0, 0, 0, 0};
static bool  s_initialized = false;

// Motor indexes (same as CF-Drone)
#define MOTOR_REAR_LEFT   0
#define MOTOR_REAR_RIGHT  1
#define MOTOR_FRONT_RIGHT 2
#define MOTOR_FRONT_LEFT  3

static int motor_get_duty(float value) {
    if (value < 0) value = 0;
    if (value > 1) value = 1;
    int max_duty = (1 << BOARD_MOTOR_PWM_BITS) - 1;
    return (int)roundf(value * max_duty);
}

void motors_init(void) {
    ESP_LOGI(TAG, "Initializing motors...");

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BOARD_MOTOR_PWM_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = BOARD_MOTOR_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 4; i++) {
        // Pull GPIO low first to prevent motor glitch on boot
        gpio_set_direction(MOTOR_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_PINS[i], 0);

        ledc_channel_config_t ch = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = MOTOR_PINS[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        esp_err_t err = ledc_channel_config(&ch);
        ESP_LOGI(TAG, "  Motor%d pin=GPIO%d: %s",
                 i, MOTOR_PINS[i], err == ESP_OK ? "OK" : "FAIL");
    }

    motors_send(); // Apply zero duty
    s_initialized = true;
    ESP_LOGI(TAG, "Motors initialized. PWM=%dHz, bits=%d",
             BOARD_MOTOR_PWM_FREQ, BOARD_MOTOR_PWM_BITS);
}

void motors_set(const float m[4]) {
    for (int i = 0; i < 4; i++) {
        s_motors[i] = m[i] < 0 ? 0 : (m[i] > 1 ? 1 : m[i]);
    }
}

void motors_send(void) {
    if (!s_initialized) return;
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i,
                      motor_get_duty(s_motors[i]));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

void motors_stop(void) {
    memset(s_motors, 0, sizeof(s_motors));
    motors_send();
}

bool motors_active(void) {
    return s_motors[0] != 0 || s_motors[1] != 0 ||
           s_motors[2] != 0 || s_motors[3] != 0;
}

const float* motors_get(void) {
    return s_motors;
}
