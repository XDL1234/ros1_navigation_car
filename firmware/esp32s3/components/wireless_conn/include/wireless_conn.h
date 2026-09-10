/**
 * @file wireless_conn.h
 * @brief WiFi无线通信与ROS节点数据交换模块头文件
 * 
 * 定义了ESP32与上位机ROS节点通信的核心数据结构和API接口
 * 支持实时双向数据传输、激光雷达转发和差速驱动控制
 * 
 * @version 1.0  
 * @date 2024
 */

#pragma once

#include "esp_err.h"
#include "mpu6050.h"
#include "controller.h"

/**
 * @brief 电机控制上下文结构体
 * 
 * 管理左右两个电机的控制句柄，支持差速驱动
 */
typedef struct {
    mc_handle_t left;       ///< 左电机控制句柄
    mc_handle_t right;      ///< 右电机控制句柄  
} motor_control_context_t;

/**
 * @brief MPU6050传感器上下文结构体
 * 
 * 包含传感器句柄、校准数据和当前读数
 */
typedef struct {
    mpu6050_handle_t sensor;            ///< MPU6050传感器句柄
    mpu6050_raw_acce_value_t acce_cal;  ///< 加速度校准值
    mpu6050_raw_gyro_value_t gyro_cal;  ///< 陀螺仪校准值
    mpu6050_raw_acce_value_t curr_acce; ///< 当前加速度读数
    mpu6050_raw_gyro_value_t curr_gyro; ///< 当前陀螺仪读数
} mpu6050_context_t;

/**
 * @brief 全局上下文包结构体
 * 
 * 整合所有硬件模块的上下文信息和机器人物理参数
 */
typedef struct {
    motor_control_context_t mc;     ///< 电机控制上下文
    mpu6050_context_t mpu6050;      ///< MPU6050传感器上下文
    float wheel_space;              ///< 轮距（mm）
    float wheel_perimeter;          ///< 轮子周长（mm）
} context_pack_t;

/**
 * @brief 启动无线通信服务
 * 
 * 初始化并启动所有网络通信任务：
 * - TCP连接管理服务（端口8080）
 * - UDP底盘数据交换（端口8082）  
 * - UDP激光雷达转发（端口8083）
 * 
 * @param ctx 全局上下文指针
 * @return esp_err_t ESP_OK成功，其他值表示失败
 */
esp_err_t start_wireless_conn(context_pack_t *ctx);

/**
 * @brief 停止无线通信服务
 * 
 * 清理所有网络资源、关闭任务和释放内存
 * 
 * @return esp_err_t ESP_OK成功，其他值表示失败
 */
esp_err_t stop_wireless_conn(void);