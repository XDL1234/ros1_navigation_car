idf_component_register(
    SRCS "wireless_conn.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos UART json mdns mpu6050 proto_data
)
