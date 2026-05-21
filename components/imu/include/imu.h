#pragma once
#include <stdbool.h>
#include "drone_types.h"

// ============================================================
// IMU Driver - MPU9250 via SPI (ESP-IDF)
// Tương đương imu.ino của CF-Drone
//
// NOTE: Nếu bạn không có MPU9250, driver này sẽ trả về
//       dữ liệu zeroed. Thay thế bằng driver phù hợp với
//       sensor của bạn trong imu.c
// ============================================================

/**
 * @brief Initialize IMU (SPI + MPU9250 config)
 * @return true if successful
 */
bool imu_init(void);

/**
 * @brief Read latest IMU data (blocking until data ready or timeout)
 * @param[out] data  Gyro (rad/s) và Acc (m/s^2) in body frame
 * @return true if new data available
 */
bool imu_read(imu_data_t *data);

/**
 * @brief Calibrate gyro bias (called when drone is stationary)
 * Must be called periodically; internally uses a slow LPF.
 */
void imu_calibrate_gyro(const imu_data_t *raw);

/**
 * @brief Print IMU info to console
 */
void imu_print_info(void);
