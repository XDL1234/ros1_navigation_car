#include <math.h>
#include <unistd.h>
#include "esp_log.h"
#include "mpu6050.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "wireless_conn.h"
#include "driver/i2c.h"

/**
 * @brief 扫描I2C总线上的设备
 *
 * @param port I2C端口号
 */
static void i2c_scan_devices(i2c_port_t port) {
    ESP_LOGI("mpu6050", "Scanning I2C bus on port %d...", port);
    int devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI("mpu6050", "Found device at address 0x%02X", addr);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        ESP_LOGW("mpu6050", "No I2C devices found on the bus!");
    } else {
        ESP_LOGI("mpu6050", "Found %d I2C device(s)", devices_found);
    }
}

/**
 * @brief 初始化MPU6050硬件
 * 
 * 该函数用于初始化MPU6050传感器硬件，包括：
 * 1. 创建MPU6050实例
 * 2. 配置加速度计和陀螺仪量程
 * 3. 唤醒设备并获取设备ID
 * 4. 设置加速度计和陀螺仪的校准值
 * 
 * @param ctx MPU6050上下文指针
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool mpu6050_hardware_init(mpu6050_context_t *ctx)
{
    static const i2c_port_t port = I2C_NUM_0;
    esp_err_t ret;

    ESP_LOGI("mpu6050", "Starting MPU6050 initialization on I2C port %d", port);

    // 扫描I2C设备
    i2c_scan_devices(port);

    // 创建MPU6050实例
    ctx->sensor = mpu6050_create(port, MPU6050_I2C_ADDRESS);
    if (ctx->sensor == NULL) {
        ESP_LOGE("mpu6050", "Failed to create MPU6050 instance");
        return false;
    }

    // 配置MPU6050
    ret = mpu6050_config(ctx->sensor, ACCE_FS_4G, GYRO_FS_1000DPS);
    if (ret != ESP_OK) {
        ESP_LOGE("mpu6050", "MPU6050 config failed: %s", esp_err_to_name(ret));
        return false;
    }

    // 唤醒MPU6050
    ret = mpu6050_wake_up(ctx->sensor);
    if (ret != ESP_OK) {
        ESP_LOGE("mpu6050", "MPU6050 wake up failed: %s", esp_err_to_name(ret));
        return false;
    }

    // 读取设备ID验证通信
    uint8_t deviceid;
    ret = mpu6050_get_deviceid(ctx->sensor, &deviceid);
    if (ret != ESP_OK) {
        ESP_LOGE("mpu6050", "Failed to read device ID: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI("mpu6050", "device id: 0x%X", deviceid);
    ctx->acce_cal.raw_acce_x = 460;
    ctx->acce_cal.raw_acce_y = 83;
    ctx->acce_cal.raw_acce_z = 7373;
    ctx->gyro_cal.raw_gyro_x = -69;
    ctx->gyro_cal.raw_gyro_y = 57;
    ctx->gyro_cal.raw_gyro_z = -16;

    return true;
}
void mpu6050_sample_task(void *pvParameters)
{
    mpu6050_context_t *ctx = pvParameters;
    for (;;) {
        if (mpu6050_get_raw_acce(ctx->sensor, &ctx->curr_acce) != ESP_OK) {
            ESP_LOGW(__func__, "update acce failed");
        }
        if (mpu6050_get_raw_gyro(ctx->sensor, &ctx->curr_gyro) != ESP_OK) {
            ESP_LOGW(__func__, "update gyro failed");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void  mpu6050_task(mpu6050_context_t *ctx)
{
    xTaskCreatePinnedToCore(mpu6050_sample_task, "mpu6050_sample",
        4096, ctx, 10, NULL,0);
}