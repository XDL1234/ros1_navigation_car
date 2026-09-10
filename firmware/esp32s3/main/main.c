
/**
 * @file main.c
 * @brief ESP32 ROS智能小车主程序
 * 
 * 本程序实现了一个基于ESP32-S3的ROS智能小车系统，主要功能包括：
 * - 双轮差速驱动控制（支持TB6612FNG和DRV8833驱动）
 * - MPU6050六轴传感器数据采集
 * - WiFi网络通信与ROS节点数据交换
 * - YDLidar X2激光雷达数据转发
 * - OLED显示屏状态显示
 * - 基于PID的闭环速度控制
 * 
 * @version 1.0
 * @date 2024
 */

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include <sys/socket.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "controller.h"
#include "proto_data.h"
#include "wireless_conn.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "esp32_i2c_rw.h"
#include "oled.h"
#include "wifi.h"
#include "UART.h"
#include "app_mpu6050.h"
#include "math.h"

// ========================= 系统配置参数 =========================

/**
 * @brief I2C设备配置
 * 用于MPU6050传感器和OLED显示屏通信
 */
i2c_device_config_t i2c_device_config = {
    .scl_pin = 42,      // I2C时钟线引脚
    .sda_pin = 41,      // I2C数据线引脚  
    .i2c_num = 0,       // 使用I2C0控制器
};

// 机械参数配置
#define MOTOR_WHEEL_SPACING        128.6    // 轮距，单位：mm
#define PLUSE_PER_ROUND            1040     // 电机转动一圈产生的脉冲数量：减速比13*20*编码器4倍频
#define MOTOR_WHEEL_CIRCLE         150.8    // 轮子周长，单位：mm (直径48mm)

// 电机驱动选择（可选TB6612FNG或DRV8833）
#define USE_TB6612_DRIVER

// PID控制器参数
#define PID_P   1.15    // 比例系数
#define PID_I   1.1     // 积分系数  
#define PID_D   0.01    // 微分系数

/**
 * @brief 全局上下文结构体
 * 包含所有硬件组件的配置和状态信息
 */
static context_pack_t ctx = {
    .wheel_space = MOTOR_WHEEL_SPACING,
    .wheel_perimeter = MOTOR_WHEEL_CIRCLE,
};


/**
 * @brief 电机控制系统初始化
 * 
 * 初始化双轮差速驱动系统，包括：
 * - PID控制器配置
 * - 编码器设置
 * - 电机驱动器配置（TB6612FNG或DRV8833）
 * - 定时器循环创建
 * 
 * @param ctx 电机控制上下文指针
 */
static void mc_init(motor_control_context_t *ctx) {
    // 创建定时器循环：PID控制周期100ms，编码器读取周期100ms
    mc_timer_loop_create(100, 100);

    // 配置PID控制器和编码器参数
    mc_config_t config = MC_CONFIG_DEFAULT(PID_P, PID_I, PID_D);
    config.encoder_config.round_count = PLUSE_PER_ROUND;  // 设置编码器一圈脉冲数

#ifdef USE_TB6612_DRIVER
    ESP_LOGI("main", "Initializing motors with TB6612 driver");

    // ========== 左电机配置 (TB6612模式) ==========
    // TB6612FNG驱动芯片配置：使用PWM+方向控制模式
    config.motor_config.pwma_gpio_num = GPIO_NUM_4;     // PWM信号引脚
    config.motor_config.pwmb_gpio_num = GPIO_NUM_37;    // 虚拟PWM引脚（TB6612不使用）
    config.tb6612_config.pwm_gpio = GPIO_NUM_4;         // TB6612实际PWM控制引脚
    config.tb6612_config.dir1_gpio = GPIO_NUM_35;       // 方向控制引脚1 (IN1)
    config.tb6612_config.dir2_gpio = GPIO_NUM_36;       // 方向控制引脚2 (IN2)
    config.encoder_config.edge_gpio_num = GPIO_NUM_6;   // 编码器A相信号
    config.encoder_config.level_gpio_num = GPIO_NUM_7;  // 编码器B相信号

    // 创建左电机控制实例，最大RPM为150
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->left));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->left));

    // ========== 右电机配置 (TB6612模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_5;     // PWM信号引脚
    config.motor_config.pwmb_gpio_num = GPIO_NUM_38;    // 虚拟PWM引脚
    config.tb6612_config.pwm_gpio = GPIO_NUM_5;         // TB6612实际PWM控制引脚
    config.tb6612_config.dir1_gpio = GPIO_NUM_47;       // 方向控制引脚1 (IN1)
    config.tb6612_config.dir2_gpio = GPIO_NUM_48;       // 方向控制引脚2 (IN2)
    config.encoder_config.edge_gpio_num = GPIO_NUM_15;  // 编码器A相信号
    config.encoder_config.level_gpio_num = GPIO_NUM_16; // 编码器B相信号
    
    // 创建右电机控制实例，最大RPM为150
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->right));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->right));

#else
    ESP_LOGI("main", "Initializing motors with DRV8833 driver");

    // ========== 左电机配置 (DRV8833模式) ==========
    // DRV8833驱动芯片配置：使用双PWM控制模式
    config.motor_config.pwma_gpio_num = GPIO_NUM_36;    // PWM A控制引脚
    config.motor_config.pwmb_gpio_num = GPIO_NUM_35;    // PWM B控制引脚
    config.encoder_config.edge_gpio_num = GPIO_NUM_19;  // 编码器A相信号
    config.encoder_config.level_gpio_num = GPIO_NUM_20; // 编码器B相信号
    
    // 创建左电机控制实例
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->left));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->left));

    // ========== 右电机配置 (DRV8833模式) ==========
    config.motor_config.pwma_gpio_num = GPIO_NUM_34;    // PWM A控制引脚
    config.motor_config.pwmb_gpio_num = GPIO_NUM_33;    // PWM B控制引脚
    config.encoder_config.edge_gpio_num = GPIO_NUM_14;  // 编码器A相信号
    config.encoder_config.level_gpio_num = GPIO_NUM_12; // 编码器B相信号
    
    // 创建右电机控制实例
    ESP_ERROR_CHECK(mc_new(&config, 150, &ctx->right));
    ESP_ERROR_CHECK(mc_add_to_timer_loop(ctx->right));
#endif

    // 启动PID控制循环，开始电机速度闭环控制
    mc_timer_loop_start();
    ESP_LOGI("main", "Motor control system started with %s driver", MOTOR_DRIVER_TYPE);
}

/**
 * @brief 硬件模块初始化函数
 * 
 * 按顺序初始化所有硬件模块，包括：
 * 1. WiFi网络模块
 * 2. I2C总线配置
 * 3. OLED显示屏
 * 4. MPU6050六轴传感器
 * 5. UART串口（用于激光雷达通信）
 * 6. 显示网络连接状态
 * 
 * @return true 所有硬件初始化成功
 * @return false 任意硬件初始化失败
 */
bool hardware_init(void)
{
    static char host[22];                    // 存储IP地址字符串
    static char buf[21] = "STAT:WIFI OK";    // WiFi状态显示字符串

    // 按照依赖关系顺序初始化各个硬件模块
    if (!wifi_init())                                    // 初始化WiFi模块
        return false;
    if (!set_i2c_device_config(&i2c_device_config))     // 配置I2C设备参数
        return false;
    if (!i2c_device_init())                             // 初始化I2C总线
        return false;
    if (!oled_init())                                   // 初始化OLED显示屏
        return false;
    if (!mpu6050_hardware_init(&ctx.mpu6050))           // 初始化MPU6050传感器
        return false;
    if (!UART_HARDWARE_init())                          // 初始化UART串口
        return false;   

    // 检查WiFi连接状态并在OLED上显示
    if (get_wifi_ip(host) != WIFI_STATUS_STA_DISCONECTED)
    {  
        oled_ascii8(0, 1, buf);     // 显示WiFi状态
        oled_ascii8(0, 2, host);    // 显示IP地址
    }
    printf("硬件初始化完成\n");
    return true;
}

/**
 * @brief ESP32应用程序主入口函数
 * 
 * 系统启动流程：
 * 1. 等待硬件初始化完成
 * 2. 初始化电机控制系统
 * 3. 启动传感器数据采集任务
 * 4. 启动显示任务
 * 5. 启动串口调试任务
 * 6. 启动网络通信任务
 * 
 * 各任务运行在FreeRTOS多任务环境中，实现并发处理：
 * - 电机PID控制（100ms周期）
 * - MPU6050数据采集
 * - OLED状态显示
 * - 激光雷达数据转发
 * - ROS节点通信
 */
void app_main(void)
{
    printf("ROS car starting\n");

    // 硬件初始化循环，直到所有硬件模块初始化成功
    while(hardware_init() == false);

    // 启动各个功能模块和任务
    mc_init(&ctx.mc);                    // 初始化电机控制系统（PID + 编码器）
    mpu6050_task(&ctx.mpu6050);          // 启动MPU6050传感器数据采集任务
    oled_show_task(&ctx);                // 启动OLED显示任务
    uart_task(&ctx);                     // 启动串口调试任务
    start_wireless_conn(&ctx);           // 启动WiFi网络通信和ROS数据交换

    printf("任务初始化完成\n");
    
    // 主任务完成初始化后进入空闲状态，由FreeRTOS调度器管理各子任务
}
