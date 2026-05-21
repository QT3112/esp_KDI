#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "drone_types.h"

// ============================================================
// RC SBUS Receiver Driver (ESP-IDF UART)
// Tương đương rc.ino của CF-Drone
//
// SBUS Protocol: 100kbps, 8E2, inverted logic, 25 bytes/frame
// ============================================================

/**
 * @brief Initialize SBUS receiver on UART
 */
void rc_sbus_init(void);

/**
 * @brief Read latest RC data (non-blocking)
 * @param[out] out  Normalized RC input [-1..1] or [0..1]
 * @return true if new frame was received
 */
bool rc_sbus_read(rc_input_t *out);

/**
 * @brief Get raw channel values [172..1811]
 * @param[out] channels  Array of 16 channel values
 */
void rc_sbus_get_raw(uint16_t channels[16]);

/**
 * @brief Print RC calibration table
 */
void rc_sbus_print_calibration(void);
