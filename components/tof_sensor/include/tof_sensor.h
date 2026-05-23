#pragma once

#include <stdbool.h>
#include <stdint.h>

// ============================================================
// TOF Sensor Driver - VL53L1X qua I2C
// SDA = GPIO 16, SCL = GPIO 17
// Tốc độ đo: ~20Hz (budget 33ms, period 40ms)
// Tầm đo: chế độ SHORT (tối đa ~1.3m, phù hợp bay thấp)
// ============================================================

/**
 * @brief Khởi tạo I2C và cảm biến VL53L1X.
 *        Phải gọi trước khi tạo tof_sensor_task.
 * @return true nếu khởi tạo thành công
 */
bool tof_sensor_init(void);

/**
 * @brief Lấy độ cao đo gần nhất (đã qua LPF), đơn vị mét.
 *        Hàm này thread-safe, có thể gọi từ bất kỳ task nào.
 * @param[out] alt_m  Giá trị độ cao (m). Trả về -1.0f nếu chưa có dữ liệu hợp lệ.
 * @return true nếu giá trị hợp lệ (RangeStatus == 0)
 */
bool tof_sensor_get_altitude(float *alt_m);

/**
 * @brief Kiểm tra cảm biến có đang hoạt động bình thường không.
 *        Trả về false nếu cảm biến mất kết nối hoặc lỗi liên tiếp.
 */
bool tof_sensor_is_valid(void);

/**
 * @brief FreeRTOS task: Vòng lặp đọc TOF ~20Hz (non-blocking poll).
 *        Tạo bằng xTaskCreatePinnedToCore() trong main.c.
 * @param arg  Không dùng (NULL)
 */
void tof_sensor_task(void *arg);
