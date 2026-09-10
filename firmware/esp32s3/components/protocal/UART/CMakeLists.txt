idf_component_register(
    SRCS "uart_dma.c" "UART.c"
    INCLUDE_DIRS "." "include"
    REQUIRES driver freertos motor ring_buffer wireless_conn
)
