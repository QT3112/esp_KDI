#pragma once
#include "drone_types.h"
#include "rc_sbus.h"

// ============================================================
// Flight Control - Cascaded PID + Safety
// Tương đương control.ino + safety.ino của CF-Drone
// ============================================================

/**
 * @brief Initialize flight controller (PID gains, limits)
 */
void fc_init(void);

/**
 * @brief Main control update - call every loop cycle
 * @param fs   Flight state (read/write)
 * @param rc   Latest RC input
 */
void fc_update(flight_state_t *fs, const rc_input_t *rc);

/**
 * @brief Get computed motor values after control update
 * @param[out] motors  Array of 4 values [RL, RR, FR, FL]
 */
void fc_get_motors(float motors[4]);

/**
 * @brief Check if armed
 */
bool fc_is_armed(void);

/**
 * @brief Reset all PID integrators (call on disarm)
 */
void fc_reset_pids(void);

/**
 * @brief Reset only the altitude hold PID integrator.
 *        Gọi khi chuyển ra khỏi chế độ ALTHOLD.
 */
void fc_reset_alt_pid(void);

/**
 * @brief Get current flight mode name string
 */
const char* fc_get_mode_name(flight_mode_t mode);
