#pragma once
#include <stdbool.h>
#include "drone_types.h"

// ============================================================
// Motor Control via LEDC (ESP-IDF)
// Tương đương motors.ino của CF-Drone
//
// Thứ tự motor: RL=0, RR=1, FR=2, FL=3
// Layout nhìn từ trên:
//   FL(3) ^ FR(2)
//         |
//   RL(0)   RR(1)
// ============================================================

/**
 * @brief Initialize LEDC and motor pins
 */
void motors_init(void);

/**
 * @brief Set motor thrust values (normalized [0..1])
 * @param m  Array of 4 values: [RL, RR, FR, FL]
 */
void motors_set(const float m[4]);

/**
 * @brief Apply motor values to LEDC hardware
 */
void motors_send(void);

/**
 * @brief Stop all motors immediately
 */
void motors_stop(void);

/**
 * @brief Check if any motor is active (>0)
 */
bool motors_active(void);

/**
 * @brief Get current motor values
 */
const float* motors_get(void);
