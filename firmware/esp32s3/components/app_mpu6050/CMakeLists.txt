idf_component_register(
    SRCS "app_mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES driver mpu6050 wireless_conn
)
