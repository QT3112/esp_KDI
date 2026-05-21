idf_component_register(
    SRCS "led_ctrl.c"
    INCLUDE_DIRS "include"
    REQUIRES "drone_config" "battery" "driver" "esp_timer"
)
