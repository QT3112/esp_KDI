#include "flight_control.h"
#include "math_utils.h"
#include "battery.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FC";

// ============================================================
// PID Gains (mirrored from control.ino of CF-Drone)
// ============================================================
// Inner loop: Rate PIDs
#define ROLLRATE_P      0.06f
#define ROLLRATE_I      0.20f
#define ROLLRATE_D      0.002f
#define ROLLRATE_I_LIM  0.35f
#define PITCHRATE_P     0.05f
#define PITCHRATE_I     0.20f
#define PITCHRATE_D     0.001f
#define PITCHRATE_I_LIM 0.35f
#define YAWRATE_P       0.30f
#define YAWRATE_I       0.01f
#define YAWRATE_D       0.01f
#define YAWRATE_I_LIM   0.30f
// Outer loop: Attitude PIDs
#define ROLL_P          7.0f
#define PITCH_P         7.0f
#define YAW_P           3.0f
// D-term LPF alpha (~40Hz cutoff)
#define RATES_D_LPF     0.2f
// ============================================================
// Limits
// ============================================================
#define PITCHRATE_MAX   (360.0f * M_PI / 180.0f)
#define ROLLRATE_MAX    (360.0f * M_PI / 180.0f)
#define YAWRATE_MAX     (300.0f * M_PI / 180.0f)
#define TILT_MAX        (30.0f  * M_PI / 180.0f)
#define MOT_THR_MIN     0.10f
#define MOT_THR_MAX     1.00f
#define ARM_THR_LIMIT   0.05f   // Max throttle for arming

// ============================================================
// Static state
// ============================================================
static pid_t s_roll_rate_pid, s_pitch_rate_pid, s_yaw_rate_pid;
static pid_t s_roll_pid, s_pitch_pid, s_yaw_pid;
static float s_motors[4] = {0};
static bool s_is_armed = false;

// Safety state
static float s_rc_loss_timeout   = 1.0f;   // seconds
static float s_descend_time      = 10.0f;  // seconds to descend to zero
static float s_rc_last_valid_t   = 0.0f;

#define INVERTED_COS_THRESHOLD  -0.7f
#define INVERTED_TIMEOUT_S       1.5f

// ============================================================
// Helper: clamp a float
// ============================================================
static inline float fc_clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// Motor desaturation (from CF-Drone desaturate())
// ============================================================
static void desaturate(float *a, float *b, float *c, float *d) {
    float mx = *a;
    if (*b > mx) mx = *b;
    if (*c > mx) mx = *c;
    if (*d > mx) mx = *d;
    if (mx > 1.0f) {
        float diff = mx - 1.0f;
        *a -= diff; *b -= diff; *c -= diff; *d -= diff;
    }
}

// ============================================================
// Initialization
// ============================================================
void fc_init(void) {
    pid_init(&s_roll_rate_pid,  ROLLRATE_P,  ROLLRATE_I,  ROLLRATE_D,
             ROLLRATE_I_LIM,  RATES_D_LPF, 0.1f);
    pid_init(&s_pitch_rate_pid, PITCHRATE_P, PITCHRATE_I, PITCHRATE_D,
             PITCHRATE_I_LIM, RATES_D_LPF, 0.1f);
    pid_init(&s_yaw_rate_pid,   YAWRATE_P,   YAWRATE_I,   YAWRATE_D,
             YAWRATE_I_LIM,   1.0f,         0.1f);
    pid_init(&s_roll_pid,  ROLL_P,  0, 0, 0, 1.0f, 0.1f);
    pid_init(&s_pitch_pid, PITCH_P, 0, 0, 0, 1.0f, 0.1f);
    pid_init(&s_yaw_pid,   YAW_P,   0, 0, 0, 1.0f, 0.1f);
    memset(s_motors, 0, sizeof(s_motors));
    ESP_LOGI(TAG, "Flight controller initialized");
}

void fc_reset_pids(void) {
    pid_reset(&s_roll_rate_pid);
    pid_reset(&s_pitch_rate_pid);
    pid_reset(&s_yaw_rate_pid);
    pid_reset(&s_roll_pid);
    pid_reset(&s_pitch_pid);
    pid_reset(&s_yaw_pid);
}

bool fc_is_armed(void) {
    return s_is_armed;
}

// ============================================================
// interpret_controls: translate RC inputs to flight targets
// (mirrors interpretControls() in control.ino)
// ============================================================
static void interpret_controls(flight_state_t *fs, const rc_input_t *rc) {
    // Flight mode from mode channel
    float mode_val = rc->mode;
    if      (mode_val < 0.25f) fs->mode = FLIGHT_MODE_ACRO;
    else if (mode_val < 0.75f) fs->mode = FLIGHT_MODE_STAB;
    else                       fs->mode = FLIGHT_MODE_ALTHOLD;

    // Arm/disarm gestures (throttle low + yaw full right/left)
    if (rc->throttle < 0.05f && rc->yaw > 0.95f) {
        float vbat = battery_get_voltage();
        if (vbat < VBAT_ABSENT_THRESHOLD || vbat >= VBAT_WARN_THRESHOLD) {
            if (!fs->armed) {
                fs->armed = true;
                ESP_LOGI(TAG, "ARMED");
                fc_reset_pids();
            }
        } else {
            ESP_LOGW(TAG, "Low battery (%.2fV), arming blocked", vbat);
        }
    }
    if (rc->throttle < 0.05f && rc->yaw < -0.95f) {
        if (fs->armed) {
            fs->armed = false;
            ESP_LOGI(TAG, "DISARMED");
            fc_reset_pids();
        }
    }

    // Web RC Buttons logic (override)
    if (rc->buttons & (1 << 0)) { // Button 0 = Arm
        float vbat = battery_get_voltage();
        if (vbat < VBAT_ABSENT_THRESHOLD || vbat >= VBAT_WARN_THRESHOLD) {
            if (!fs->armed) {
                fs->armed = true;
                ESP_LOGI(TAG, "ARMED via Web");
                fc_reset_pids();
            }
        }
    }
    if (rc->buttons & (1 << 1)) { // Button 1 = Disarm
        if (fs->armed) {
            fs->armed = false;
            ESP_LOGI(TAG, "DISARMED via Web");
            fc_reset_pids();
        }
    }
    if (rc->buttons & (1 << 2)) { // Button 2 = Emergency Stop
        fs->armed = false;
        fs->thrust_target = 0.0f;
    }

    s_is_armed = fs->armed;

    // Web RC Buttons logic (override)
    if (rc->buttons & (1 << 0)) { // Button 0 = Arm
        float vbat = battery_get_voltage();
        if (vbat < VBAT_ABSENT_THRESHOLD || vbat >= VBAT_WARN_THRESHOLD) {
            if (!fs->armed) {
                fs->armed = true;
                ESP_LOGI(TAG, "ARMED via Web");
                fc_reset_pids();
            }
        }
    }
    if (rc->buttons & (1 << 1)) { // Button 1 = Disarm
        if (fs->armed) {
            fs->armed = false;
            ESP_LOGI(TAG, "DISARMED via Web");
            fc_reset_pids();
        }
    }
    if (rc->buttons & (1 << 2)) { // Button 2 = Emergency Stop
        fs->armed = false;
        fs->thrust_target = 0.0f;
    }

    // Yaw dead zone
    float yaw = fabsf(rc->yaw) < 0.1f ? 0 : rc->yaw;

    // Thrust
    if (rc->throttle < 0.05f) {
        fs->thrust_target = 0.0f;
    } else {
        fs->thrust_target = mapf(rc->throttle, 0.05f, 1.0f,
                                 MOT_THR_MIN, MOT_THR_MAX);
    }

    if (fs->mode == FLIGHT_MODE_STAB) {
        float yaw_target = quat_get_yaw(fs->attitude_target);
        if (!fs->armed || !isfinite(yaw_target) || yaw != 0) {
            yaw_target = quat_get_yaw(fs->attitude);
        }
        fs->attitude_target = quat_from_euler(
            rc->roll  * TILT_MAX,
            rc->pitch * TILT_MAX,
            yaw_target);
        fs->rates_extra = (vec3_t){0, 0, -yaw * YAWRATE_MAX};
    } else if (fs->mode == FLIGHT_MODE_ACRO) {
        quat_invalidate(&fs->attitude_target);
        fs->rates_target = (vec3_t){
             rc->roll  * ROLLRATE_MAX,
             rc->pitch * PITCHRATE_MAX,
            -yaw       * YAWRATE_MAX
        };
    } else if (fs->mode == FLIGHT_MODE_RAW) {
        quat_invalidate(&fs->attitude_target);
        vec3_invalidate(&fs->rates_target);
        fs->torque_target = (vec3_t){
            rc->roll  * 0.1f,
            rc->pitch * 0.1f,
           -yaw       * 0.1f
        };
    }

    // Track RC last valid time
    if (rc->valid) s_rc_last_valid_t = fs->t;
}

// ============================================================
// control_attitude: outer PID loop
// (mirrors controlAttitude() in control.ino)
// ============================================================
static void control_attitude(flight_state_t *fs) {
    if (!fs->armed || !quat_valid(fs->attitude_target) ||
        fs->thrust_target < MOT_THR_MIN) return;

    const vec3_t up = {0, 0, 1};
    vec3_t up_actual = quat_rotate_vector(up, fs->attitude);
    vec3_t up_target = quat_rotate_vector(up, fs->attitude_target);

    vec3_t error = vec3_rotation_between(up_target, up_actual);

    float rx = pid_update(&s_roll_pid,  error.x, fs->t);
    float ry = pid_update(&s_pitch_pid, error.y, fs->t);

    float yaw_err = wrap_angle(quat_get_yaw(fs->attitude_target) -
                                quat_get_yaw(fs->attitude));
    float rz = pid_update(&s_yaw_pid, yaw_err, fs->t);

    fs->rates_target = (vec3_t){
        rx + fs->rates_extra.x,
        ry + fs->rates_extra.y,
        rz + fs->rates_extra.z
    };
}

// ============================================================
// control_rates: inner PID loop
// (mirrors controlRates() in control.ino)
// ============================================================
static void control_rates(flight_state_t *fs) {
    if (!fs->armed || !vec3_valid(fs->rates_target) ||
        fs->thrust_target < MOT_THR_MIN) return;

    vec3_t error = vec3_sub(fs->rates_target, fs->rates);

    fs->torque_target = (vec3_t){
        pid_update(&s_roll_rate_pid,  error.x, fs->t),
        pid_update(&s_pitch_rate_pid, error.y, fs->t),
        pid_update(&s_yaw_rate_pid,   error.z, fs->t)
    };
}

// ============================================================
// control_torque: mix torques to motor thrusts
// (mirrors controlTorque() in control.ino)
//
// Motor layout (top view, X-frame):
//   FL(3) ^ FR(2)
//         |
//   RL(0)   RR(1)
// ============================================================
static void control_torque(flight_state_t *fs) {
    if (!vec3_valid(fs->torque_target)) return;

    if (!fs->armed) {
        memset(s_motors, 0, sizeof(s_motors));
        return;
    }

    if (fs->thrust_target < MOT_THR_MIN) {
        for (int i = 0; i < 4; i++) s_motors[i] = MOT_THR_MIN;
        return;
    }

    vec3_t t = fs->torque_target;
    float T  = fs->thrust_target;

    // X-frame motor mixing (CF-Drone convention):
    // roll(x): +FL, +RL, -FR, -RR
    // pitch(y): +FR, +FL... wait, CF-Drone: FL = T + roll - pitch + yaw
    s_motors[3] = T + t.x - t.y + t.z; // FL
    s_motors[2] = T - t.x - t.y - t.z; // FR
    s_motors[0] = T + t.x + t.y - t.z; // RL
    s_motors[1] = T - t.x + t.y + t.z; // RR

    desaturate(&s_motors[3], &s_motors[2], &s_motors[0], &s_motors[1]);

    for (int i = 0; i < 4; i++) {
        s_motors[i] = fc_clamp(s_motors[i], 0.0f, 1.0f);
    }
}

// ============================================================
// Safety - Failsafe checks
// (mirrors safety.ino of CF-Drone)
// ============================================================
static void safety_rc_loss(flight_state_t *fs) {
    if (s_rc_last_valid_t == 0) return; // Never received RC
    if (!fs->armed) return;
    if (fs->t - s_rc_last_valid_t > s_rc_loss_timeout) {
        // Smooth descent
        fs->mode = FLIGHT_MODE_AUTO;
        fs->attitude_target = quat_identity();
        fs->thrust_target -= fs->dt / s_descend_time;
        if (fs->thrust_target < 0) {
            fs->thrust_target = 0;
            fs->armed = false;
            ESP_LOGW(TAG, "RC lost → auto-disarmed");
        }
    }
}

static void safety_inverted(flight_state_t *fs) {
    if (!fs->armed) { fs->is_inverted = false; return; }

    const vec3_t world_up = {0, 0, 1};
    vec3_t up = quat_rotate_vector(world_up, fs->attitude);

    static float inverted_start = 0;
    if (up.z < INVERTED_COS_THRESHOLD) {
        fs->is_inverted = true;
        if (inverted_start == 0) inverted_start = fs->t;
        if (fs->t - inverted_start > INVERTED_TIMEOUT_S) {
            fs->armed = false;
            fs->thrust_target = 0;
            inverted_start = 0;
            ESP_LOGW(TAG, "Inverted protection: disarmed");
        }
    } else {
        fs->is_inverted = false;
        inverted_start = 0;
    }
}

static void safety_battery(flight_state_t *fs) {
    float vbat = battery_get_voltage();
    if (vbat < VBAT_ABSENT_THRESHOLD) return;
    if (!fs->armed) return;

    bool flying = fs->thrust_target >= BATTERY_FLYING_THRUST_MIN;
    if (flying && vbat < VBAT_CRITICAL_THRESHOLD) {
        // Descend
        fs->mode = FLIGHT_MODE_AUTO;
        fs->attitude_target = quat_identity();
        fs->thrust_target -= fs->dt / s_descend_time;
        if (fs->thrust_target < 0) { fs->thrust_target = 0; fs->armed = false; }
    } else if (!flying && vbat < VBAT_WARN_THRESHOLD) {
        // Auto-lock at idle
        fs->armed = false;
        fs->thrust_target = 0;
        ESP_LOGW(TAG, "Low battery (%.2fV): auto-disarmed", vbat);
    }
}

// ============================================================
// Main FC update
// ============================================================
void fc_update(flight_state_t *fs, const rc_input_t *rc) {
    // 1. Interpret RC inputs → flight targets
    interpret_controls(fs, rc);

    // 2. Safety checks (failsafe)
    safety_rc_loss(fs);
    safety_inverted(fs);
    safety_battery(fs);

    // 3. Cascaded PID
    control_attitude(fs); // Outer: attitude → rates target
    control_rates(fs);    // Inner: rates → torques
    control_torque(fs);   // Mixer: torques → motor values
}

void fc_get_motors(float motors[4]) {
    for (int i = 0; i < 4; i++) motors[i] = s_motors[i];
}

const char* fc_get_mode_name(flight_mode_t mode) {
    switch (mode) {
        case FLIGHT_MODE_RAW:     return "RAW";
        case FLIGHT_MODE_ACRO:    return "ACRO";
        case FLIGHT_MODE_STAB:    return "STAB";
        case FLIGHT_MODE_ALTHOLD: return "ALTHOLD";
        case FLIGHT_MODE_AUTO:    return "AUTO";
        default:                  return "UNKNOWN";
    }
}
