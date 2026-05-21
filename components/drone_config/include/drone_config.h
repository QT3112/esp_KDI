#pragma once

// ============================================================
// Board-Level Hardware Configuration for ESP-IDF
// Tương đương board_config.h của CF-Drone (Arduino)
// ============================================================

// ---- I2C (VL53L1X ToF Sensor) ----
#define BOARD_I2C_PORT          0
#define BOARD_I2C_SDA_PIN       17
#define BOARD_I2C_SCL_PIN       18
#define BOARD_I2C_FREQ_HZ       400000

// ---- SPI (IMU - MPU9250/6500) ----
#define BOARD_SPI_HOST          SPI2_HOST
#define BOARD_SPI_MOSI_PIN      23      // D23: MOSI
#define BOARD_SPI_MISO_PIN      19      // D19: MISO
#define BOARD_SPI_SCK_PIN       18      // D18: SCK
#define BOARD_IMU_CS_PIN        5       // D5:  NCS (Chip Select)

// ---- Motor PWM (LEDC) ----
// Thứ tự: Rear-Left=0, Rear-Right=1, Front-Right=2, Front-Left=3
#define BOARD_MOTOR_PIN_RL      12
#define BOARD_MOTOR_PIN_RR      13
#define BOARD_MOTOR_PIN_FR      15
#define BOARD_MOTOR_PIN_FL      14
#define BOARD_MOTOR_PWM_FREQ    25000   // 25kHz MOSFET mode
#define BOARD_MOTOR_PWM_BITS    10      // 10-bit resolution

// ---- RC Receiver (SBUS via UART) ----
#define BOARD_RC_UART_NUM       UART_NUM_1
#define BOARD_RC_RX_PIN         4
#define BOARD_RC_TX_PIN         -1      // TX không dùng (SBUS chỉ nhận)

// ---- Battery ADC ----
#define BOARD_VBAT_ADC_CHANNEL  ADC_CHANNEL_0  // GPIO36 = VP = ADC1_CH0
#define BOARD_VBAT_DIVIDER      (43.0f / 33.0f) // R-up 10k, R-down 33k
#define BOARD_VBAT_ADC_SAMPLES  16

// ---- LED ----
#define BOARD_LED_PIN           2       // Built-in LED GPIO2
#define BOARD_LED_ENABLED       1
