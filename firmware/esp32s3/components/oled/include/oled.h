/**
 * @file oled.h
 * @brief SSD1306 OLED显示屏控制模块头文件
 * 
 * 定义了基于I2C通信的SSD1306 OLED显示屏控制接口，主要功能包括：
 * - 128x64像素单色OLED显示控制
 * - ASCII字符显示支持
 * - 坐标定位和绘制函数
 * - FreeRTOS任务驱动的实时状态显示
 * 
 * @version 1.0
 * @date 2024
 */

#pragma once
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp32_i2c_rw.h"
#include "wireless_conn.h"

// ========================= 常量定义 =========================

#define OLED_ADDR 0x3C  ///< SSD1306 OLED设备I2C地址（七位地址）

// ========================= 数据结构定义 =========================

/**
 * @brief OLED显示行结构体
 * 
 * 用于描述OLED显示屏上一行的显示数据
 * 注意：该结构体定义但在当前代码中未使用
 */
typedef struct
{
    uint8_t x;          ///< X轴坐标（列位置）0-127
    uint8_t y;          ///< Y轴坐标（页面位置）0-7
    uint8_t row[128];   ///< 一行的像素数据（128列）
} oled_show_line_t;

// ========================= 函数声明 =========================

/**
 * @brief 初始化SSD1306 OLED显示屏
 * 
 * 配置SSD1306控制器的各项参数并启动显示：
 * - 设置显示分辨率128x64
 * - 配置对比度和亮度
 * - 启用电荷泵和显示
 * - 清空屏幕并显示标题
 * 
 * 前提条件：I2C总线已初始化
 * 
 * @return true 初始化成功
 * @return false 初始化失败（当前实现总是返回true）
 */
bool oled_init(void);

/**
 * @brief 清空OLED显示屏
 * 
 * 将整个OLED显示屏的所有像素清零，显示为全黑屏幕
 * 使用页面寻址模式遍历所有8页，每页128列
 */
void oled_clear(void);

/**
 * @brief 设置OLED显示坐标
 * 
 * 坐标系统定义：
 * 0-----------127 (x轴，列坐标)
 * |
 * |
 * 7 (y轴，页面坐标)
 * 
 * @param x 列坐标（0-127），指定水平像素位置
 * @param y 页面坐标（0-7），每页包含8行像素
 */
void oled_setxy(uint8_t x, uint8_t y);

/**
 * @brief 自动换行显示ASCII字符串
 * 
 * 提供类似终端输出的自动滚动显示功能：
 * - 自动管理行位置递增
 * - 超出屏幕范围时自动循环
 * - 支持最多21个字符的字符串
 * 
 * @param str 要显示的ASCII字符串（最多21个字符）
 * @return true 显示成功
 * @return false 字符串超出长度限制
 */
bool oled_show_ascii_auto_line(char *str);

/**
 * @brief 显示ASCII字符串（大字体）
 * 
 * 显示8x16像素的双行大字体ASCII字符串：
 * - 每个字符占用8列×16行像素
 * - 最多显示16个字符
 * - 占用两个相邻的OLED页面（上下两行）
 * 
 * @param x 起始列坐标（0-127）
 * @param y 起始页面坐标（0-6）
 * @param str ASCII字符串
 */
void oled_ascii(uint8_t x, uint8_t y, char *str);

/**
 * @brief 显示ASCII字符串（小字体）
 * 
 * 显示6x8像素的单行小字体ASCII字符串：
 * - 每个字符占用6列×8行像素
 * - 最多显示21个字符
 * - 仅占用一个OLED页面（单行）
 * - 适合显示详细数据和状态信息
 * 
 * @param x 起始列坐标（0-127）
 * @param y 页面坐标（0-7）
 * @param str ASCII字符串
 */
void oled_ascii8(uint8_t x, uint8_t y, char *str);

/**
 * @brief 创建OLED实时显示任务
 * 
 * 启动一个高优先级的FreeRTOS任务用于实时更新OLED显示内容：
 * - 显示MPU6050陀螺仪数据
 * - 显示PID控制器参数
 * - 显示左右电机转速状态
 * - 显示机器人线速度信息
 * - 100ms刷新周期，提供流畅的显示体验
 * 
 * 任务特性：
 * - 优先级：10（高优先级）
 * - 栈大小：8KB
 * - CPU核心：0
 * 
 * @param ctx 全局上下文指针，包含电机控制和传感器数据
 */
void oled_show_task(context_pack_t *ctx);
