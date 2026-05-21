#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

#define I2C_PORT_NUM 0
#define I2C_TIMEOUT_MS 1000

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t buffer[count + 2];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    memcpy(&buffer[2], pdata, count);

    esp_err_t err = i2c_master_write_to_device(I2C_PORT_NUM, Dev->I2cDevAddr >> 1, buffer, count + 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err == ESP_OK ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t reg[2];
    reg[0] = (index >> 8) & 0xFF;
    reg[1] = index & 0xFF;
    
    esp_err_t err = i2c_master_write_read_device(I2C_PORT_NUM, Dev->I2cDevAddr >> 1, reg, 2, pdata, count, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err == ESP_OK ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
    uint8_t buffer[3];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    buffer[2] = data;

    esp_err_t err = i2c_master_write_to_device(I2C_PORT_NUM, Dev->I2cDevAddr >> 1, buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err == ESP_OK ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
    uint8_t buffer[4];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;

    esp_err_t err = i2c_master_write_to_device(I2C_PORT_NUM, Dev->I2cDevAddr >> 1, buffer, 4, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err == ESP_OK ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
    uint8_t buffer[6];
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    buffer[2] = (data >> 24) & 0xFF;
    buffer[3] = (data >> 16) & 0xFF;
    buffer[4] = (data >> 8) & 0xFF;
    buffer[5] = data & 0xFF;

    esp_err_t err = i2c_master_write_to_device(I2C_PORT_NUM, Dev->I2cDevAddr >> 1, buffer, 6, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err == ESP_OK ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t data;
    VL53L1_Error status = VL53L1_RdByte(Dev, index, &data);
    if (status != VL53L1_ERROR_NONE) { return status; }
    data &= AndData;
    data |= OrData;
    return VL53L1_WrByte(Dev, index, data);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 2);
    if (status == VL53L1_ERROR_NONE) {
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 4);
    if (status == VL53L1_ERROR_NONE) {
        *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    }
    return status;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
    *ptick_count_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    return VL53L1_ERROR_NOT_IMPLEMENTED;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(wait_ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
    esp_rom_delay_us(wait_us); // Note: esp_rom_delay_us may block CPU, use with caution for long delays. 
    // But for typical VL53L1X short waits, it's fine.
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
    VL53L1_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    uint8_t data;
    VL53L1_Error status;

    uint32_t start_time_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while ((uint32_t)(esp_timer_get_time() / 1000) - start_time_ms < timeout_ms)
    {
        status = VL53L1_RdByte(pdev, index, &data);
        if (status != VL53L1_ERROR_NONE) { return status; }
        if ((data & mask) == value) { return VL53L1_ERROR_NONE; }
        TickType_t ticks = pdMS_TO_TICKS(poll_delay_ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }

    return VL53L1_ERROR_TIME_OUT;
}
