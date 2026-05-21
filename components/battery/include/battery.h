#pragma once
#include <stdbool.h>

// ============================================================
// Battery Voltage Monitor (ADC)
// Tương đương battery.ino của CF-Drone
// ============================================================

#define VBAT_WARN_THRESHOLD       3.5f   // L1: Idle low-battery lock warning
#define VBAT_LOW_THRESHOLD        3.3f   // L2: In-flight LED warning
#define VBAT_CRITICAL_THRESHOLD   3.0f   // L3: Auto-land
#define VBAT_ABSENT_THRESHOLD     0.5f   // Below this: no battery
#define BATTERY_FLYING_THRUST_MIN 0.15f  // Thrust >= this means in-flight
#define BATTERY_DEBOUNCE_TIME_S   0.9f   // Anti-bounce time (seconds)

/**
 * @brief Initialize ADC for battery voltage
 */
void battery_init(void);

/**
 * @brief Update battery voltage reading (call at ~2Hz)
 */
void battery_update(void);

/**
 * @brief Get latest battery voltage in volts
 */
float battery_get_voltage(void);
