# CF-Drone ESP-IDF Firmware Architecture

Tài liệu này mô tả kiến trúc tổng quan của phần mềm điều khiển bay CF-Drone được port sang nền tảng ESP-IDF (Sử dụng FreeRTOS).

## 1. Kiến trúc phần cứng (Hardware Block Diagram)

Biểu đồ dưới đây mô tả cách ESP32 kết nối với các thiết bị ngoại vi trên Drone:

```mermaid
graph TD
    ESP32[ESP32 Microcontroller]
    IMU[MPU9250 IMU]
    SBUS[RC Receiver - SBUS]
    MOTORS[BLDC Motors x4]
    BATT[Battery ADC]
    LED[Status LED]
    TOF[VL53L1X ToF Sensor]

    ESP32 <-->|SPI2| IMU
    SBUS -->|UART1 RX| ESP32
    ESP32 -->|LEDC PWM| MOTORS
    BATT -->|ADC1 CH0| ESP32
    ESP32 -->|GPIO| LED
    ESP32 <-->|I2C0| TOF
```

- **IMU**: Kết nối qua SPI tốc độ cao để đảm bảo độ trễ thấp nhất.
- **SBUS**: Kết nối qua UART1 (chỉ dùng chân RX), cấu hình đảo mức logic tín hiệu ngay bên trong UART driver của ESP32.
- **MOTORS**: Điều khiển bằng module LEDC của ESP-IDF với tần số PWM 25kHz.
- **ToF**: Cảm biến đo khoảng cách kết nối qua I2C.

---

## 2. Kiến trúc phần mềm (Software Block Diagram)

Hệ thống được chia thành 2 task chính chạy song song trên 2 nhân (core) của ESP32 để tối ưu hiệu suất và đảm bảo tính thời gian thực (Real-time).

```mermaid
graph TD
    subgraph FreeRTOS
        subgraph Core 0
            DT[drone_task<br>1000Hz Fast Loop]
        end
        subgraph Core 1
            AT[aux_task<br>10Hz Slow Loop]
        end
    end

    subgraph Hardware Drivers
        IMU_DRV[imu.c]
        SBUS_DRV[rc_sbus.c]
        MOTORS_DRV[motors.c]
        BATT_DRV[battery.c]
        LED_DRV[led_ctrl.c]
    end

    subgraph Core Logic
        ATT[attitude_estimator.c]
        FC[flight_control.c]
        MATH[math_utils.h]
    end

    IMU_DRV --> DT
    SBUS_DRV --> DT
    DT --> ATT
    ATT --> FC
    FC --> MOTORS_DRV

    BATT_DRV --> AT
    AT --> LED_DRV
```

- **Core 0 (`drone_task`)**: Đảm nhiệm các tác vụ đòi hỏi độ chính xác cao về mặt thời gian như đọc cảm biến IMU, tính toán góc nghiêng, chạy bộ điều khiển PID và xuất xung điều khiển động cơ.
- **Core 1 (`aux_task`)**: Xử lý các tác vụ ít quan trọng hơn như đọc điện áp pin, điều khiển đèn LED nháy cảnh báo và in log ra màn hình.

---

## 3. Lưu đồ thuật toán điều khiển bay (Flight Control Flowchart)

Dưới đây là chi tiết vòng lặp chính của `drone_task`. Vòng lặp này chạy với chu kỳ khoảng 1ms (tương đương 1000Hz).

```mermaid
flowchart TD
    Start([Bắt đầu drone_task]) --> ReadIMU[1. Đọc IMU<br>SPI Polling]
    ReadIMU --> StepTime[2. Cập nhật Delta Time]
    StepTime --> CheckLand{3. Đang ở mặt đất?}
    CheckLand -- Đúng --> Calibrate[Hiệu chỉnh Gyro Bias]
    CheckLand -- Sai --> ReadRC[4. Đọc tín hiệu điều khiển RC<br>Từ UART Buffer]
    Calibrate --> ReadRC
    ReadRC --> EstAtt[5. Ước lượng tư thế<br>Cập nhật Quaternion]
    EstAtt --> FC[6. Điều khiển bay<br>Tính toán PID, Mix động cơ]
    FC --> SetMotors[7. Cập nhật tốc độ 4 động cơ]
    SetMotors --> Yield[8. vTaskDelay 1 tick<br>Reset Watchdog]
    Yield --> ReadIMU
```

### Chú thích chi tiết vòng lặp:
1. **Đọc IMU**: Driver đọc dữ liệu từ MPU9250 qua SPI. Bước này có thời gian xử lý rất nhanh nhờ tốc độ SPI 20MHz.
2. **Cập nhật thời gian**: Tính `dt` (thời gian trôi qua so với vòng lặp trước) để dùng cho các phép tích phân.
3. **Hiệu chỉnh Gyro**: Nếu drone chưa được arm (động cơ chưa quay) và đang nằm im, bộ lọc sẽ liên tục tìm và bù trừ sai số (bias) của Gyroscope.
4. **Đọc RC**: Hàm đọc sẽ lấy frame dữ liệu mới nhất (nếu có) từ bộ đệm của UART. Không làm block task.
5. **Ước lượng tư thế**: Sử dụng dữ liệu Gyro và Accel để cập nhật góc nghiêng (Roll, Pitch) và hướng (Yaw) thông qua Quaternion.
6. **Điều khiển bay**: Xử lý logic an toàn (Arm/Disarm), Desaturation, và chạy bộ PID vòng ngoài (Angle) + vòng trong (Rate).
7. **Cập nhật động cơ**: Đẩy dữ liệu ra module LEDC để thay đổi độ rộng xung PWM ngay lập tức.
8. **Delay 1 tick**: Block task trong 1ms để đảm bảo các task hệ thống (IDLE) của FreeRTOS được chạy, tránh lỗi Task Watchdog Timeout.
