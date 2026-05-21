idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    REQUIRES "drone_config" "math_utils" "driver"
)
