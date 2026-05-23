idf_component_register(
    SRCS "tof_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES "vl53l1x-st-api" "driver" "freertos" "esp_timer"
)
