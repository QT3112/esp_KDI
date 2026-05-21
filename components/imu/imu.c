/*
 * IMU Driver: MPU9250 / MPU6500 via SPI (ESP-IDF)
 *
 * Pins (configured via drone_config.h):
 *   MISO = D19, SCK = D18, MOSI = D23, NCS = D5
 *
 * Data output:
 *   gyro  = angular rates in rad/s
 *   acc   = acceleration  in m/s^2
 *
 * Register reference: PS-MPU-9250A-01, InvenSense datasheet
 */

#include "imu.h"
#include "drone_config.h"
#include "math_utils.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "IMU";

// ============================================================
// MPU9250 Register Map
// ============================================================
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_ACCEL_XOUT_H    0x3B    // First data register (6 bytes ACC, 2 TEMP, 6 GYRO)
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_USER_CTRL       0x6A
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C
#define MPU_REG_WHO_AM_I        0x75

#define MPU9250_WHO_AM_I_VAL    0x71
#define MPU6500_WHO_AM_I_VAL    0x70

// SPI read/write bit
#define SPI_READ                0x80

// Sensitivity constants
#define GYRO_SCALE_2000DPS      (2000.0f / 32768.0f * M_PI / 180.0f)   // rad/s per LSB
#define ACC_SCALE_4G            (4.0f / 32768.0f * 9.80665f)           // m/s^2 per LSB

// ============================================================
// SPI handle
// ============================================================
static spi_device_handle_t s_spi = NULL;

// Calibration
static vec3_t s_gyro_bias = {0, 0, 0};
static vec3_t s_acc_bias  = {0, 0, 0};
static vec3_t s_acc_scale = {1, 1, 1};
static lpf_vec3_t s_gyro_bias_lpf;
static bool s_initialized = false;

// Mounting rotation: adjust these if sensor PCB is rotated
// Example: if sensor is mounted 180° around Z-axis:
//   s_imu_rotation_yaw = M_PI
#define IMU_MOUNT_ROLL   0.0f
#define IMU_MOUNT_PITCH  0.0f
#define IMU_MOUNT_YAW    0.0f

// ============================================================
// Low-level SPI helpers
// ============================================================

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data) {
    spi_transaction_t t = {
        .length    = 16,       // 8-bit reg + 8-bit data = 16 bits
        .tx_buffer = (uint8_t[]){reg & 0x7F, data},
        .rx_buffer = NULL,
    };
    return spi_device_transmit(s_spi, &t);
}

static esp_err_t mpu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0xFF, sizeof(tx));
    tx[0] = reg | SPI_READ;

    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK) {
        memcpy(data, rx + 1, len);
    }
    return err;
}

static uint8_t mpu_read_byte(uint8_t reg) {
    uint8_t val = 0;
    mpu_read_reg(reg, &val, 1);
    return val;
}

// ============================================================
// Initialization
// ============================================================

bool imu_init(void) {
    lpf_vec3_init(&s_gyro_bias_lpf, 0.001f);

    // 1. Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num   = BOARD_SPI_MOSI_PIN,
        .miso_io_num   = BOARD_SPI_MISO_PIN,
        .sclk_io_num   = BOARD_SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // SPI_DMA_DISABLED: MPU9250 reads are only 15 bytes max, DMA is not needed.
    // DMA mode uses a background ISR bus-lock mechanism that requires IDLE task to run,
    // which causes Task Watchdog timeouts in a tight polling loop.
    esp_err_t err = spi_bus_initialize(BOARD_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    // 2. Add MPU9250 device (SPI Mode 3: CPOL=1, CPHA=1)
    // Use 1MHz for init register writes (some boards are slow at startup)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode           = 3,
        .spics_io_num   = BOARD_IMU_CS_PIN,
        .queue_size     = 4,
    };
    err = spi_bus_add_device(BOARD_SPI_HOST, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return false;
    }

    // 3. Check WHO_AM_I
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t who = mpu_read_byte(MPU_REG_WHO_AM_I);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X or 0x%02X)",
             who, MPU9250_WHO_AM_I_VAL, MPU6500_WHO_AM_I_VAL);
    if (who != MPU9250_WHO_AM_I_VAL && who != MPU6500_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU9250/6500 not detected! Check wiring and SPI config.");
        return false;
    }

    // 4. Reset device
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);  // Device reset
    vTaskDelay(pdMS_TO_TICKS(100));

    // 5. Wake up, auto select best clock source
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);  // Clock source: PLL with gyro X
    vTaskDelay(pdMS_TO_TICKS(10));

    // 6. Enable all acc+gyro axes
    mpu_write_reg(MPU_REG_PWR_MGMT_2, 0x00);

    // 7. Sample rate divider: SAMPLE_RATE = Gyro_Rate / (1 + SMPLRT_DIV)
    //    Gyro internal rate = 1kHz when DLPF enabled
    //    SMPLRT_DIV = 0 → 1kHz output
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0);

    // 8. DLPF config: ~92Hz bandwidth for gyro
    mpu_write_reg(MPU_REG_CONFIG, 0x02);

    // 9. Gyro range: ±2000 °/s
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x18);

    // 10. Acc range: ±4g
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x08);

    // 11. Acc DLPF: ~99Hz bandwidth
    mpu_write_reg(MPU_REG_ACCEL_CONFIG2, 0x02);

    // 12. Enable data-ready interrupt
    mpu_write_reg(MPU_REG_INT_ENABLE, 0x01);

    // 13. After init, raise SPI clock to 20 MHz for fast data reading
    //     Remove and re-add device with higher clock speed
    ESP_ERROR_CHECK(spi_bus_remove_device(s_spi));
    devcfg.clock_speed_hz = 20 * 1000 * 1000;  // 20 MHz (MPU9250 SPI max = 20MHz)
    ESP_ERROR_CHECK(spi_bus_add_device(BOARD_SPI_HOST, &devcfg, &s_spi));

    s_initialized = true;
    ESP_LOGI(TAG, "MPU9250/6500 initialized. Gyro=±2000dps, Acc=±4g, Rate=1kHz");
    return true;
}

// ============================================================
// Read IMU data
// ============================================================

bool imu_read(imu_data_t *data) {
    if (!s_initialized) {
        data->valid = false;
        return false;
    }

    // Read 14 bytes: ACCEL_XOUT_H through GYRO_ZOUT_L
    // Layout: AX_H AX_L AY_H AY_L AZ_H AZ_L TEMP_H TEMP_L GX_H GX_L GY_H GY_L GZ_H GZ_L
    uint8_t buf[14];
    esp_err_t err = mpu_read_reg(MPU_REG_ACCEL_XOUT_H, buf, 14);
    if (err != ESP_OK) {
        data->valid = false;
        return false;
    }

    // Parse raw 16-bit signed values
    int16_t ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperature (ignored here)
    int16_t gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to physical units
    vec3_t acc_raw = {
        ax_raw * ACC_SCALE_4G,
        ay_raw * ACC_SCALE_4G,
        az_raw * ACC_SCALE_4G,
    };
    vec3_t gyro_raw = {
        gx_raw * GYRO_SCALE_2000DPS,
        gy_raw * GYRO_SCALE_2000DPS,
        gz_raw * GYRO_SCALE_2000DPS,
    };

    // Apply mounting rotation (NOP if all zeros)
    if (IMU_MOUNT_ROLL != 0 || IMU_MOUNT_PITCH != 0 || IMU_MOUNT_YAW != 0) {
        quaternion_t rot = quat_from_euler(IMU_MOUNT_ROLL, IMU_MOUNT_PITCH, IMU_MOUNT_YAW);
        quaternion_t inv = quat_inverse(rot);
        acc_raw  = quat_rotate_vector(acc_raw,  inv);
        gyro_raw = quat_rotate_vector(gyro_raw, inv);
    }

    // Apply accelerometer calibration
    data->acc.x = (acc_raw.x - s_acc_bias.x) / s_acc_scale.x;
    data->acc.y = (acc_raw.y - s_acc_bias.y) / s_acc_scale.y;
    data->acc.z = (acc_raw.z - s_acc_bias.z) / s_acc_scale.z;

    // Apply gyro bias subtraction
    data->gyro  = vec3_sub(gyro_raw, s_gyro_bias);
    data->valid = true;

    return true;
}

// ============================================================
// Gyro bias calibration (call while stationary)
// ============================================================

void imu_calibrate_gyro(const imu_data_t *raw) {
    // Slowly track gyro bias via LPF while drone is on ground
    s_gyro_bias = lpf_vec3_update(&s_gyro_bias_lpf, raw->gyro);
}

// ============================================================
// Debug info
// ============================================================

void imu_print_info(void) {
    ESP_LOGI(TAG, "MPU9250/6500 SPI Driver");
    ESP_LOGI(TAG, "Pins: MOSI=%d MISO=%d SCK=%d CS=%d",
             BOARD_SPI_MOSI_PIN, BOARD_SPI_MISO_PIN,
             BOARD_SPI_SCK_PIN,  BOARD_IMU_CS_PIN);
    ESP_LOGI(TAG, "Gyro bias: %.4f %.4f %.4f rad/s",
             s_gyro_bias.x, s_gyro_bias.y, s_gyro_bias.z);
    ESP_LOGI(TAG, "Acc  bias: %.4f %.4f %.4f m/s^2",
             s_acc_bias.x, s_acc_bias.y, s_acc_bias.z);
}
