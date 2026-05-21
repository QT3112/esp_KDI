idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS ""
    PRIV_REQUIRES
        "esp_timer"
        "drone_config"
        "math_utils"
        "imu"
        "rc_sbus"
        "motors"
        "battery"
        "led_ctrl"
        "attitude_estimator"
        "flight_control"
        "wifi_ap"
        "web_rc"
)
