#pragma once
#include "drone_types.h"
#include "imu.h"

// ============================================================
// Attitude Estimator
// Tương đương estimate.ino của CF-Drone
// Input: IMU data (gyro, acc)
// Output: Quaternion attitude, filtered rates
// ============================================================

/**
 * @brief Initialize attitude estimator
 */
void attitude_estimator_init(void);

/**
 * @brief Update attitude estimation - call at IMU rate (1kHz)
 * @param imu    Latest IMU data (gyro rad/s, acc m/s^2)
 * @param fs     Flight state to update (attitude, rates, landed)
 */
void attitude_estimator_update(const imu_data_t *imu, flight_state_t *fs);
