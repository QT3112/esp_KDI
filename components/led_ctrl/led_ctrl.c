#include "drone_config.h"
#include "drone_types.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "battery.h"


#if BOARD_LED_ENABLED

#define BLINK_SLOW_US    500000   // 500ms half-period → 1 Hz
#define BLINK_FAST_US     62500   // 62.5ms half-period → 8 Hz

static bool s_led_state = false;

static void led_set_raw(bool on) {
    if (on == s_led_state) return;
    gpio_set_level(BOARD_LED_PIN, on ? 1 : 0);
    s_led_state = on;
}

void led_ctrl_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOARD_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_set_raw(false);
}

void led_ctrl_set(bool on) {
    led_set_raw(on);
}

void led_ctrl_update(const flight_state_t *fs) {
    int64_t us = esp_timer_get_time();

    if (!fs->armed) {
        if (fs->battery_voltage > VBAT_ABSENT_THRESHOLD &&
            fs->battery_voltage < VBAT_WARN_THRESHOLD) {
            led_set_raw((us / BLINK_FAST_US) % 2); // Low battery before arm: fast blink
        } else {
            led_set_raw(false); // Normal idle: OFF
        }
        return;
    }

    // Armed: check alerts
    bool any_alert = false;
    if (fs->is_inverted) any_alert = true;
    if (fs->battery_voltage > VBAT_ABSENT_THRESHOLD &&
        fs->battery_voltage < VBAT_LOW_THRESHOLD) any_alert = true;

    if (any_alert) {
        led_set_raw((us / BLINK_FAST_US) % 2); // Alert: fast blink 8Hz
    } else {
        led_set_raw((us / BLINK_SLOW_US) % 2); // Normal flight: slow blink 1Hz
    }
}

#else
// Board without LED
void led_ctrl_init(void) {}
void led_ctrl_set(bool on) { (void)on; }
void led_ctrl_update(const flight_state_t *fs) { (void)fs; }
#endif
