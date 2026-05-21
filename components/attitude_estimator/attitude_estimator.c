#include "attitude_estimator.h"
#include "math_utils.h"
#include "motors.h"
#include <math.h>

// Weights for accelerometer correction (from CF-Drone estimate.ino)
#define ACC_WEIGHT   0.003f
#define LEVEL_WEIGHT 0.0002f
#define ONE_G        9.80665f

// Low-pass filter for angular rates (cutoff ~40 Hz, alpha=0.2)
#define RATES_LPF_ALPHA 0.2f

static lpf_vec3_t s_rates_filter;
static bool       s_initialized = false;

void attitude_estimator_init(void) {
    lpf_vec3_init(&s_rates_filter, RATES_LPF_ALPHA);
    s_initialized = false; // attitude will be initialized on first update
}

// Apply gyro integration to update attitude
static void apply_gyro(flight_state_t *fs, const vec3_t *gyro) {
    // Filter gyro → angular rates
    fs->rates = lpf_vec3_update(&s_rates_filter, *gyro);

    if (!s_initialized) {
        // Initialize attitude to identity on first call
        fs->attitude = quat_identity();
        s_initialized = true;
        return;
    }

    // Integrate rates × dt → rotation vector → quaternion increment
    vec3_t rot = vec3_scale(fs->rates, fs->dt);
    quaternion_t delta = quat_from_rotation_vector(rot);
    fs->attitude = quat_rotate(fs->attitude, delta);
}

// Apply accelerometer gravity correction (when landed/stationary)
static void apply_acc(flight_state_t *fs, const vec3_t *acc) {
    float acc_norm = vec3_norm(*acc);

    // Detect if landed: motors off AND acceleration ≈ 1G
    fs->landed = !motors_active() &&
                 fabsf(acc_norm - ONE_G) < ONE_G * 0.1f;

    if (!fs->landed) return;

    // Gravity correction
    const vec3_t up_world = {0, 0, 1};
    vec3_t up_actual = quat_rotate_vector(up_world, fs->attitude);
    vec3_t correction = vec3_rotation_between(*acc, up_actual);
    correction = vec3_scale(correction, ACC_WEIGHT);

    quaternion_t corr_q = quat_from_rotation_vector(correction);
    fs->attitude = quat_rotate(fs->attitude, corr_q);
}

// Apply level correction in flight (pilot assumed to keep drone mostly level)
static void apply_level(flight_state_t *fs) {
    if (fs->landed) return;

    const vec3_t up_world  = {0, 0, 1};
    vec3_t up_actual = quat_rotate_vector(up_world, fs->attitude);
    vec3_t correction = vec3_rotation_between(up_world, up_actual);
    correction = vec3_scale(correction, LEVEL_WEIGHT);

    quaternion_t corr_q = quat_from_rotation_vector(correction);
    fs->attitude = quat_rotate(fs->attitude, corr_q);
}

void attitude_estimator_update(const imu_data_t *imu, flight_state_t *fs) {
    if (!imu->valid || fs->dt <= 0) return;

    apply_gyro(fs, &imu->gyro);
    apply_acc(fs, &imu->acc);
    apply_level(fs);
}
