#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "wireless_conn.h"
bool mpu6050_hardware_init(mpu6050_context_t *ctx);
void mpu6050_sample_task(void *pvParameters);
void  mpu6050_task(mpu6050_context_t *ctx);

#ifdef __cplusplus
}
#endif