#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)//强制结构体/联合体的内存 1 字节对齐

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vec3_t;

typedef union {
    struct {
        uint8_t head;     /*!< 0x7B */
        uint8_t reserve;  /*!< blank */
        vec3_t velocity;  /*!< velocity */
        vec3_t acce;       /*!< acceleration */
        vec3_t gyro;      /*!< gyroscope */
        int16_t power;    /*!< power voltage (mV) */
        uint8_t checksum; /*!< xor value before checksum */
        uint8_t tail;     /*!< 0x7D */
    };
    uint8_t buffer[0];
} ros_send_data_frame_t;

typedef union {
    struct {
        uint8_t head;     /*!< 0x7B */
        uint16_t reserve; /*!< blank */
        vec3_t velocity;  /*!< velocity multiply 1000 */
#ifdef PID_DEBUG
        struct {
            float kp;
            float ki;
            float kd;
        } pid;
#endif // PID_DEBUG
        uint8_t checksum; /*!< xor value before checksum */
        uint8_t tail;     /*!< 0x7D */
    };
    uint8_t buffer[0];
} ros_recv_data_frame_t;

typedef union {
    struct {
        uint8_t head0;    /*!< 0xBB */
        uint8_t head1;    /*!< 0xBB */
        uint8_t function; /*!< 1 */
        uint8_t len;      /*!< 4 */
        int8_t loffset;
        int8_t roffset;
        uint8_t checksum; /*!< sum before this */
    };
    uint8_t buffer[0];
} remote_data_frame_t;

#pragma pack()//恢复默认内存对齐

#define ROS_HEAD 0x7B
#define ROS_TAIL 0x7D

bool ros_send_data_format_check(const uint8_t *frame);
bool ros_recv_data_format_check(const uint8_t *frame);
bool rc_data_format_check(const uint8_t *frame);


#ifdef __cplusplus
}
#endif
