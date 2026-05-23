#pragma once
#include <stdbool.h>
#include <stdint.h>

// ============================================================
// Shared Types & Enums across all drone components
// ============================================================

// ---- Flight Modes ----
typedef enum {
    FLIGHT_MODE_RAW     = 0,   // Direct torque control (debug)
    FLIGHT_MODE_ACRO    = 1,   // Rate control only
    FLIGHT_MODE_STAB    = 2,   // Attitude stabilization (default)
    FLIGHT_MODE_ALTHOLD = 3,   // Altitude hold (requires baro/ToF)
    FLIGHT_MODE_AUTO    = 4    // Autonomous (failsafe descent)
} flight_mode_t;

// ---- 3-axis float vector ----
typedef struct {
    float x, y, z;
} vec3_t;

// ---- Quaternion ----
typedef struct {
    float w, x, y, z;
} quaternion_t;

// ---- IMU raw data ----
typedef struct {
    vec3_t gyro;    // rad/s
    vec3_t acc;     // m/s^2
    bool   valid;
} imu_data_t;

// ---- RC Input (normalized) ----
typedef struct {
    float roll;      // [-1, 1]
    float pitch;     // [-1, 1]
    float yaw;       // [-1, 1]
    float throttle;  // [ 0, 1]
    float mode;      // [ 0, 1]  (3-position switch)
    uint16_t buttons; // Bitmask for web rc buttons
    bool  valid;
    float last_update_s; // time of last valid frame
} rc_input_t;

// ---- Motor values ----
typedef struct {
    float m[4]; // normalized [0..1]: RL, RR, FR, FL
} motor_values_t;

// ---- Flight state (shared global) ----
typedef struct {
    flight_mode_t   mode;
    bool            armed;
    float           thrust_target;
    quaternion_t    attitude;
    quaternion_t    attitude_target;
    vec3_t          rates;           // filtered angular rates rad/s
    vec3_t          rates_target;
    vec3_t          rates_extra;     // feedforward
    vec3_t          torque_target;
    bool            landed;
    float           battery_voltage;
    bool            is_inverted;
    float           t;               // current time (s)
    float           dt;              // loop delta time (s)
    float           loop_rate;       // Hz
    // --- TOF / Altitude Hold ---
    float           alt_measured_m;  // Độ cao đo từ TOF (m), -1.0f nếu không hợp lệ
    float           alt_target_m;    // Độ cao mục tiêu khi ở chế độ ALTHOLD
    bool            tof_valid;       // Cảm biến TOF đang hoạt động bình thường
} flight_state_t;
