idf_component_register(
    SRCS "flight_control.c"
    INCLUDE_DIRS "include"
    REQUIRES "drone_config" "math_utils" "battery" "rc_sbus" "tof_sensor"
)
