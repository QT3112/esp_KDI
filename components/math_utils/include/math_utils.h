#pragma once
#include <math.h>
#include <stdbool.h>
#include "drone_types.h"

// ============================================================
// Vec3 - 3D Vector math (C version of vector.h)
// ============================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline vec3_t vec3(float x, float y, float z) {
    return (vec3_t){x, y, z};
}

static inline vec3_t vec3_zero(void) {
    return (vec3_t){0, 0, 0};
}

static inline bool vec3_valid(const vec3_t v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static inline void vec3_invalidate(vec3_t *v) {
    v->x = NAN; v->y = NAN; v->z = NAN;
}

static inline vec3_t vec3_add(vec3_t a, vec3_t b) {
    return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline vec3_t vec3_sub(vec3_t a, vec3_t b) {
    return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline vec3_t vec3_scale(vec3_t v, float s) {
    return (vec3_t){v.x * s, v.y * s, v.z * s};
}

static inline vec3_t vec3_div(vec3_t v, float s) {
    return (vec3_t){v.x / s, v.y / s, v.z / s};
}

static inline float vec3_dot(vec3_t a, vec3_t b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float vec3_norm(vec3_t v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static inline vec3_t vec3_normalize(vec3_t v) {
    float n = vec3_norm(v);
    if (n < 1e-9f) return vec3_zero();
    return vec3_div(v, n);
}

static inline vec3_t vec3_cross(vec3_t a, vec3_t b) {
    return (vec3_t){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

// Rotation vector between two vectors (axis-angle representation)
static inline vec3_t vec3_rotation_between(vec3_t a, vec3_t b) {
    float an = vec3_norm(a);
    float bn = vec3_norm(b);
    if (an < 1e-6f || bn < 1e-6f) return vec3_zero();

    vec3_t direction = vec3_cross(a, b);
    if (vec3_norm(direction) < 1e-6f) {
        if (vec3_dot(a, b) > 0) return vec3_zero();
        // Opposite direction - find perpendicular
        vec3_t perp = vec3_cross(a, vec3(1, 0, 0));
        if (vec3_norm(perp) < 1e-6f) perp = vec3_cross(a, vec3(0, 1, 0));
        return vec3_scale(vec3_normalize(perp), M_PI);
    }
    direction = vec3_normalize(direction);
    float dot_clamped = fmaxf(-1.0f, fminf(1.0f, vec3_dot(a, b) / (an * bn)));
    float angle = acosf(dot_clamped);
    return vec3_scale(direction, angle);
}

// ============================================================
// Quaternion math (C version of quaternion.h)
// ============================================================

static inline quaternion_t quat_identity(void) {
    return (quaternion_t){1, 0, 0, 0};
}

static inline bool quat_valid(quaternion_t q) {
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

static inline void quat_invalidate(quaternion_t *q) {
    q->w = NAN; q->x = NAN; q->y = NAN; q->z = NAN;
}

static inline float quat_norm(quaternion_t q) {
    return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

static inline quaternion_t quat_normalize(quaternion_t q) {
    float n = quat_norm(q);
    return (quaternion_t){q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline quaternion_t quat_multiply(quaternion_t a, quaternion_t b) {
    return (quaternion_t){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,
        a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x
    };
}

static inline quaternion_t quat_inverse(quaternion_t q) {
    float n2 = 1.0f / (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return (quaternion_t){q.w*n2, -q.x*n2, -q.y*n2, -q.z*n2};
}

// Rotate vector by quaternion: v' = q * v * q^-1
static inline vec3_t quat_rotate_vector(vec3_t v, quaternion_t q) {
    quaternion_t qv = {0, v.x, v.y, v.z};
    quaternion_t res = quat_multiply(quat_inverse(q), quat_multiply(qv, q));
    return (vec3_t){res.x, res.y, res.z};
}

// Apply incremental rotation b to quaternion a
static inline quaternion_t quat_rotate(quaternion_t a, quaternion_t b) {
    return quat_normalize(quat_multiply(a, b));
}

static inline quaternion_t quat_from_axis_angle(vec3_t axis, float angle) {
    float half = angle * 0.5f;
    float s = sinf(half) / vec3_norm(axis);
    return (quaternion_t){cosf(half), axis.x*s, axis.y*s, axis.z*s};
}

static inline quaternion_t quat_from_rotation_vector(vec3_t rot) {
    float n = vec3_norm(rot);
    if (n < 1e-9f) return quat_identity();
    return quat_from_axis_angle(rot, n);
}

// ZYX Euler angles (roll, pitch, yaw) → Quaternion
static inline quaternion_t quat_from_euler(float roll, float pitch, float yaw) {
    float cx = cosf(roll/2),  sx = sinf(roll/2);
    float cy = cosf(pitch/2), sy = sinf(pitch/2);
    float cz = cosf(yaw/2),   sz = sinf(yaw/2);
    return (quaternion_t){
        cx*cy*cz + sx*sy*sz,
        sx*cy*cz - cx*sy*sz,
        cx*sy*cz + sx*cy*sz,
        cx*cy*sz - sx*sy*cz
    };
}

// Quaternion → Euler angles
static inline vec3_t quat_to_euler(quaternion_t q) {
    float sqx = q.x*q.x, sqy = q.y*q.y, sqz = q.z*q.z, sqw = q.w*q.w;
    float sarg = -2.0f*(q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw);
    vec3_t euler;
    if (sarg <= -0.99999f) {
        euler.x = 0; euler.y = -0.5f*M_PI; euler.z = -2.0f*atan2f(q.y, q.x);
    } else if (sarg >= 0.99999f) {
        euler.x = 0; euler.y =  0.5f*M_PI; euler.z =  2.0f*atan2f(q.y, q.x);
    } else {
        euler.x = atan2f(2.0f*(q.y*q.z + q.w*q.x), sqw - sqx - sqy + sqz);
        euler.y = asinf(sarg);
        euler.z = atan2f(2.0f*(q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz);
    }
    return euler;
}

static inline float quat_get_yaw(quaternion_t q)   { return quat_to_euler(q).z; }
static inline float quat_get_pitch(quaternion_t q) { return quat_to_euler(q).y; }
static inline float quat_get_roll(quaternion_t q)  { return quat_to_euler(q).x; }

// ============================================================
// Low-pass filter
// ============================================================

typedef struct {
    float alpha;
    float output;
    bool  initialized;
} lpf_float_t;

static inline void lpf_float_init(lpf_float_t *f, float alpha) {
    f->alpha = alpha; f->initialized = false; f->output = 0;
}

static inline float lpf_float_update(lpf_float_t *f, float input) {
    if (!f->initialized) { f->output = input; f->initialized = true; return input; }
    if (f->alpha >= 1.0f) return input;
    f->output += f->alpha * (input - f->output);
    return f->output;
}

static inline void lpf_float_reset(lpf_float_t *f) {
    f->initialized = false;
}

typedef struct {
    float alpha;
    vec3_t output;
    bool   initialized;
} lpf_vec3_t;

static inline void lpf_vec3_init(lpf_vec3_t *f, float alpha) {
    f->alpha = alpha; f->initialized = false;
}

static inline vec3_t lpf_vec3_update(lpf_vec3_t *f, vec3_t input) {
    if (!f->initialized) { f->output = input; f->initialized = true; return input; }
    if (f->alpha >= 1.0f) return input;
    f->output.x += f->alpha * (input.x - f->output.x);
    f->output.y += f->alpha * (input.y - f->output.y);
    f->output.z += f->alpha * (input.z - f->output.z);
    return f->output;
}

// ============================================================
// PID Controller (C version of pid.h)
// ============================================================

typedef struct {
    float kp, ki, kd;
    float windup;       // integral windup limit (0 = disabled)
    float dt_max;       // max dt before resetting integrator
    float integral;
    float derivative;
    float prev_error;
    float prev_time;
    lpf_float_t d_lpf;  // low pass filter for derivative
} pid_t;

static inline void pid_init(pid_t *pid, float kp, float ki, float kd,
                             float windup, float d_alpha, float dt_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->windup = windup; pid->dt_max = dt_max;
    pid->integral = 0; pid->derivative = 0;
    pid->prev_error = NAN; pid->prev_time = NAN;
    lpf_float_init(&pid->d_lpf, d_alpha);
}

static inline float pid_update(pid_t *pid, float error, float t) {
    float dt = t - pid->prev_time;
    if (isfinite(dt) && dt > 0 && dt < pid->dt_max) {
        pid->integral += error * dt;
        pid->derivative = lpf_float_update(&pid->d_lpf,
                                            (error - pid->prev_error) / dt);
    } else {
        pid->integral = 0;
        pid->derivative = 0;
    }
    pid->prev_error = error;
    pid->prev_time = t;

    float integral_clamped = pid->windup > 0
        ? fmaxf(-pid->windup, fminf(pid->windup, pid->integral))
        : pid->integral;

    return pid->kp * error + pid->ki * integral_clamped + pid->kd * pid->derivative;
}

static inline void pid_reset(pid_t *pid) {
    pid->prev_error = NAN;
    pid->prev_time = NAN;
    pid->integral = 0;
    pid->derivative = 0;
    lpf_float_reset(&pid->d_lpf);
}

// ============================================================
// Utility functions
// ============================================================

static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float wrap_angle(float a) {
    a = fmodf(a, 2*M_PI);
    if (a >  M_PI) a -= 2*M_PI;
    if (a < -M_PI) a += 2*M_PI;
    return a;
}
