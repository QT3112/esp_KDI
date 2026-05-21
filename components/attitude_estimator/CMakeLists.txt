idf_component_register(
    SRCS "attitude_estimator.c"
    INCLUDE_DIRS "include"
    REQUIRES "drone_config" "math_utils" "imu" "motors"
)
