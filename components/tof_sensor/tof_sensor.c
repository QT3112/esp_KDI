#include "tof_sensor.h"
#include "vl53l1_api.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TOF";

// ============================================================
// Cấu hình phần cứng
// ============================================================
#define TOF_I2C_PORT        I2C_NUM_0
#define TOF_I2C_SDA_PIN     16
#define TOF_I2C_SCL_PIN     17
#define TOF_I2C_FREQ_HZ     400000      // 400kHz Fast Mode

// ============================================================
// Cấu hình cảm biến VL53L1X
// Budget 33ms + Period 40ms → ~25Hz thực tế
// SHORT mode: tầm đo 10cm–1.3m, ít bị ảnh hưởng ánh sáng
// ============================================================
#define TOF_TIMING_BUDGET_US    33000   // 33ms
#define TOF_INTER_MEAS_MS       40      // 40ms

// LPF alpha: 0.3 = lọc mạnh (ưu tiên ổn định), 0.7 = đáp ứng nhanh hơn
#define TOF_LPF_ALPHA           0.3f

// Số lần lỗi liên tiếp tối đa trước khi đánh dấu cảm biến không hợp lệ
#define TOF_MAX_CONSEC_ERRORS   10

// ============================================================
// Trạng thái nội bộ (shared, bảo vệ bằng critical section)
// ============================================================
static VL53L1_Dev_t     s_dev;
static VL53L1_DEV       s_Dev = &s_dev;

static volatile float   s_alt_m            = -1.0f;    // Giá trị độ cao đã LPF (m)
static volatile bool    s_tof_valid        = false;    // Cảm biến đang hoạt động tốt
static volatile int     s_consec_errors    = 0;        // Đếm lỗi liên tiếp

// Dùng portMUX_TYPE cho critical section trên ESP32
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Khởi tạo I2C Master
// ============================================================
static bool i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = TOF_I2C_SDA_PIN,
        .scl_io_num       = TOF_I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOF_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(TOF_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(TOF_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2C Master OK (SDA=GPIO%d, SCL=GPIO%d, %dkHz)",
             TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN, TOF_I2C_FREQ_HZ / 1000);
    return true;
}

// ============================================================
// Khởi tạo VL53L1X
// ============================================================
bool tof_sensor_init(void)
{
    // 1. Khởi tạo I2C
    if (!i2c_master_init()) {
        return false;
    }

    // 2. Cấu hình device struct
    memset(&s_dev, 0, sizeof(s_dev));
    s_Dev->I2cDevAddr = 0x52;  // Địa chỉ I2C mặc định 8-bit của VL53L1X

    // 3. Chờ sensor boot xong
    VL53L1_Error status = VL53L1_WaitDeviceBooted(s_Dev);
    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "WaitDeviceBooted failed: %d (Kiểm tra kết nối I2C!)", status);
        return false;
    }

    // 4. Đọc Model ID để xác nhận kết nối
    uint8_t model_id = 0;
    VL53L1_RdByte(s_Dev, 0x010F, &model_id);
    ESP_LOGI(TAG, "VL53L1X Model ID: 0x%02X (Expect 0xEA)", model_id);

    // 5. Chuỗi khởi tạo chính
    if (status == VL53L1_ERROR_NONE) status = VL53L1_DataInit(s_Dev);
    if (status == VL53L1_ERROR_NONE) status = VL53L1_StaticInit(s_Dev);

    // 6. Chế độ đo: SHORT (10cm–1.3m, phù hợp bay thấp, ít nhiễu ánh sáng)
    if (status == VL53L1_ERROR_NONE)
        status = VL53L1_SetDistanceMode(s_Dev, VL53L1_DISTANCEMODE_SHORT);

    // 7. Timing: budget 33ms, period 40ms → ~25Hz
    if (status == VL53L1_ERROR_NONE)
        status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(s_Dev, TOF_TIMING_BUDGET_US);
    if (status == VL53L1_ERROR_NONE)
        status = VL53L1_SetInterMeasurementPeriodMilliSeconds(s_Dev, TOF_INTER_MEAS_MS);

    // 8. Bắt đầu đo liên tục
    if (status == VL53L1_ERROR_NONE)
        status = VL53L1_StartMeasurement(s_Dev);

    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Init failed at step, status=%d", status);
        return false;
    }

    s_tof_valid = true;
    ESP_LOGI(TAG, "VL53L1X Init OK – SHORT mode, budget=%dms, period=%dms",
             TOF_TIMING_BUDGET_US / 1000, TOF_INTER_MEAS_MS);
    return true;
}

// ============================================================
// API công khai: lấy độ cao (thread-safe)
// Dùng taskENTER/EXIT_CRITICAL_FROM_ISR safe cross-core variant
// ============================================================
bool tof_sensor_get_altitude(float *alt_m)
{
    // Đọc các biến volatile: trên Xtensa 32-bit, float/bool read là atomic
    // nếu aligned. Dùng volatile đủ để đảm bảo không bị optimize.
    float val   = s_alt_m;
    bool  valid = s_tof_valid;

    *alt_m = val;
    return valid && (val >= 0.0f);
}

bool tof_sensor_is_valid(void)
{
    return s_tof_valid;
}

// ============================================================
// FreeRTOS Task: Vòng lặp đọc TOF non-blocking (~25Hz)
// QUAN TRỌNG: KHÔNG bao giờ gọi ESP_LOG* bên trong critical section!
// portENTER_CRITICAL tắt interrupt, nhưng ESP_LOG cần acquire mutex.
// ============================================================
void tof_sensor_task(void *arg)
{
    ESP_LOGI(TAG, "TOF task started on Core %d", xPortGetCoreID());

    static float lpf_val = -1.0f;  // Biến LPF nội bộ task (không cần bảo vệ)

    while (1) {
        uint8_t is_ready = 0;
        VL53L1_Error status = VL53L1_GetMeasurementDataReady(s_Dev, &is_ready);

        if (status != VL53L1_ERROR_NONE) {
            // Lỗi giao tiếp I2C — cập nhật shared state trong critical section
            bool should_log = false;
            portENTER_CRITICAL(&s_mux);
            s_consec_errors++;
            if (s_consec_errors >= TOF_MAX_CONSEC_ERRORS) {
                s_tof_valid = false;
                should_log  = true;  // Đánh dấu để log SAU khi thoát critical
            }
            portEXIT_CRITICAL(&s_mux);

            // Log BÊN NGOÀI critical section
            if (should_log) {
                ESP_LOGE(TAG, "Cảm biến mất kết nối (lỗi liên tiếp >= %d)", TOF_MAX_CONSEC_ERRORS);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (!is_ready) {
            // Chưa có dữ liệu mới, đợi thêm
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Có dữ liệu mới → đọc
        VL53L1_RangingMeasurementData_t meas;
        status = VL53L1_GetRangingMeasurementData(s_Dev, &meas);

        // Xóa cờ ngắt để cho phép đo lần tiếp theo
        VL53L1_ClearInterruptAndStartMeasurement(s_Dev);

        if (status != VL53L1_ERROR_NONE) {
            portENTER_CRITICAL(&s_mux);
            s_consec_errors++;
            if (s_consec_errors >= TOF_MAX_CONSEC_ERRORS) {
                s_tof_valid = false;
            }
            portEXIT_CRITICAL(&s_mux);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Chỉ cập nhật khi RangeStatus == 0 (đo hợp lệ hoàn toàn)
        if (meas.RangeStatus == 0) {
            float new_m = meas.RangeMilliMeter * 0.001f;  // mm → m

            // Áp dụng LPF: lần đầu tiên lấy thẳng giá trị
            if (lpf_val < 0.0f) {
                lpf_val = new_m;
            } else {
                lpf_val = TOF_LPF_ALPHA * new_m + (1.0f - TOF_LPF_ALPHA) * lpf_val;
            }

            // Cập nhật shared state
            portENTER_CRITICAL(&s_mux);
            s_alt_m         = lpf_val;
            s_tof_valid     = true;
            s_consec_errors = 0;
            portEXIT_CRITICAL(&s_mux);

        } else {
            // Đo không hợp lệ (tín hiệu yếu, vật ngoài tầm, v.v.)
            bool should_log = false;
            portENTER_CRITICAL(&s_mux);
            s_consec_errors++;
            if (s_consec_errors >= TOF_MAX_CONSEC_ERRORS) {
                s_tof_valid = false;
                should_log  = true;
            }
            portEXIT_CRITICAL(&s_mux);

            if (should_log) {
                ESP_LOGW(TAG, "TOF out-of-range liên tiếp >= %d lần (RangeStatus=%d)",
                         TOF_MAX_CONSEC_ERRORS, meas.RangeStatus);
            }
        }

        // Nhường CPU cho các task khác
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
