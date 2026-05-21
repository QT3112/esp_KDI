#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "vl53l1_api.h"

static const char *TAG = "VL53L1X";

#define I2C_MASTER_SCL_IO           18      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           17      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000  /*!< I2C master clock frequency */

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C Master");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    VL53L1_Dev_t dev;
    VL53L1_DEV pdev = &dev;
    pdev->I2cDevAddr = 0x52; // Default address is 0x52 (8-bit)

    VL53L1_Error status = VL53L1_ERROR_NONE;

    ESP_LOGI(TAG, "Waiting for device to boot...");
    status = VL53L1_WaitDeviceBooted(pdev);
    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to boot VL53L1X: %d", status);
        return;
    }
    ESP_LOGI(TAG, "Device booted!");

    status = VL53L1_DataInit(pdev);
    if (status != VL53L1_ERROR_NONE) { ESP_LOGE(TAG, "DataInit failed: %d", status); return; }
    
    status = VL53L1_StaticInit(pdev);
    if (status != VL53L1_ERROR_NONE) { ESP_LOGE(TAG, "StaticInit failed: %d", status); return; }
    
    // Configure distance mode to Long
    status = VL53L1_SetDistanceMode(pdev, VL53L1_DISTANCEMODE_LONG);
    if (status != VL53L1_ERROR_NONE) { ESP_LOGE(TAG, "SetDistanceMode failed: %d", status); return; }
    
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pdev, 50000);
    if (status != VL53L1_ERROR_NONE) { ESP_LOGE(TAG, "SetTimingBudget failed: %d", status); return; }
    
    // Inter-measurement period must be > TimingBudget.
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pdev, 60);
    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "SetInterMeasurementPeriod failed: %d", status);
        return;
    }

    ESP_LOGI(TAG, "Starting measurement...");
    status = VL53L1_StartMeasurement(pdev);

    while (1) {
        VL53L1_RangingMeasurementData_t RangingData;
        
        status = VL53L1_WaitMeasurementDataReady(pdev);
        if (status == VL53L1_ERROR_NONE) {
            status = VL53L1_GetRangingMeasurementData(pdev, &RangingData);
            if (status == VL53L1_ERROR_NONE) {
                if (RangingData.RangeStatus == 0) {
                    printf("Distance: %d mm\n", RangingData.RangeMilliMeter);
                } else {
                    printf("Distance: %d mm (Status: %d)\n", RangingData.RangeMilliMeter, RangingData.RangeStatus);
                }
            }
            VL53L1_ClearInterruptAndStartMeasurement(pdev);
        } else {
            ESP_LOGW(TAG, "Timeout waiting for data");
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
