/**
 * @file wireless_conn.c
 * @brief WiFi网络通信与ROS节点数据交换模块
 * 
 * 本模块负责ESP32与上位机ROS节点之间的网络通信，主要功能包括：
 * 
 * 核心通信功能：
 * - TCP连接管理（端口8080）：处理上位机连接配置
 * - UDP数据交换（端口8082）：双向传输机器人控制和状态数据
 * - UDP雷达转发（端口8083）：将激光雷达数据实时转发给ROS节点
 * 
 * 数据处理功能：
 * - 差速驱动运动学解算：速度指令转换为左右轮RPM
 * - 里程计反馈计算：从轮速计算机器人实际速度
 * - 硬件抽象层：处理对向安装轮子的方向差异
 * - 单位转换：mm/s ↔ m/s, mrad/s ↔ rad/s
 * 
 * 实时数据流：
 * - 接收：上位机速度控制指令（linear_x, angular_z）
 * - 发送：机器人状态反馈（速度、加速度、陀螺仪、电压）
 * - 转发：激光雷达扫描数据（使用DMA优化）
 * 
 * @version 1.0
 * @date 2024
 */

#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_check.h"
#include "cJSON.h"
#include "UART.h"
#include "proto_data.h"
#include "wireless_conn.h"
#include "uart_dma.h"

#define TAG "ros_conn"

// ========================= 宏定义和全局变量 =========================

#define NEXT_POW2(x) (1 << ( (8*sizeof(x) - __builtin_clz( (x | (x-1)) + 1 )) ))

// 网络套接字管理
static int g_socks[] = { -1, -1, -1, -1 };      // 套接字数组：[连接,底盘,遥控,激光雷达]
static TaskHandle_t g_task_handles[4];           // 任务句柄数组
static SemaphoreHandle_t g_chassis_send_muetx;   // 底盘数据发送互斥锁
static SemaphoreHandle_t g_chassis_recv_muetx;   // 底盘数据接收互斥锁
static SemaphoreHandle_t g_remote_muetx;         // 遥控数据互斥锁
static SemaphoreHandle_t g_lidar_send_muetx;     // 激光雷达数据发送互斥锁

// 套接字别名定义，提高代码可读性
#define g_conn_sock (g_socks[0])      // TCP连接管理套接字（端口8080）
#define g_chassis_sock (g_socks[1])   // 底盘控制UDP套接字（端口8082）
#define g_remote_sock (g_socks[2])    // 遥控器UDP套接字
#define g_lidar_sock (g_socks[3])     // 激光雷达UDP套接字（端口8083）

// 系统状态管理
static context_pack_t *g_ctx;                              // 全局上下文指针
static bool g_ros_ctrl = false;                            // ROS控制状态标志
static struct sockaddr_in g_lidar_target_addr;             // 激光雷达数据目标地址
static bool g_has_lidar_target = false;                    // 激光雷达目标地址有效标志
static struct sockaddr_in g_chassis_target_addr;           // 底盘数据目标地址


/**
 * @brief ROS数据发送任务
 * 
 * 负责周期性向上位机ROS节点发送机器人状态数据，包括：
 * - 里程计反馈（基于编码器的速度估计）
 * - IMU传感器数据（加速度、陀螺仪）
 * - 系统状态信息（电池电压等）
 * 
 * 数据处理流程：
 * 1. 读取左右轮实际RPM
 * 2. 硬件抽象：处理对向安装轮子方向
 * 3. 正运动学解算：RPM → 机器人线速度和角速度
 * 4. 单位转换：m/s → mm/s, rad/s → mrad/s
 * 5. 打包发送UDP数据帧
 * 
 * @param pvParameters UDP套接字文件描述符
 */
static void ros_send_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t temp = 6;  
    for (;;) {
        // 步骤1：读取电机编码器反馈的实际RPM
        float l_rpm_raw = mc_get_real_rpm(g_ctx->mc.left);
        float r_rpm_raw = mc_get_real_rpm(g_ctx->mc.right);
        
        // 步骤2：硬件抽象层 - 处理对向安装轮子的方向差异
        float l_rpm = -l_rpm_raw;  // 左轮对向安装，反馈时取负号匹配运动学模型
        float r_rpm = r_rpm_raw;   // 右轮正向安装
        
        // 步骤3：单位转换 - 机器人物理参数从mm转换为m
        float wheel_perimeter_m = g_ctx->wheel_perimeter / 1000.0f; // 轮子周长：mm -> m
        float wheel_space_m = g_ctx->wheel_space / 1000.0f;         // 轮距：mm -> m
        
        // 步骤4：RPM转换为线速度（m/s）
        float l_speed_ms = (l_rpm / 60.0f) * wheel_perimeter_m;  // 左轮线速度
        float r_speed_ms = (r_rpm / 60.0f) * wheel_perimeter_m;  // 右轮线速度
        
        // 步骤5：正运动学解算 - 从轮速计算机器人整体运动状态
        float linear_x_ms = (r_speed_ms + l_speed_ms) / 2.0f;           // 机器人线速度
        float angular_z_rads = (r_speed_ms - l_speed_ms) / wheel_space_m; // 机器人角速度
        
        // 步骤6：单位转换为上位机期望格式，并限制在int16_t范围内
        int16_t velocity_x_mms = (int16_t)(linear_x_ms * 1000.0f);      // m/s -> mm/s
        int16_t velocity_z_mrads = (int16_t)(angular_z_rads * 1000.0f); // rad/s -> mrad/s
        
        // 步骤7：构建ROS数据发送帧
        ros_send_data_frame_t frame = {
            .head = ROS_HEAD,                                // 数据帧头标识
            .acce = *(vec3_t *)&g_ctx->mpu6050.curr_acce,   // MPU6050加速度数据
            .gyro = *(vec3_t *)&g_ctx->mpu6050.curr_acce,   // MPU6050陀螺仪数据
            .velocity = {
                .x = velocity_x_mms,    // X轴线速度（mm/s）
                .y = 0,                 // Y轴速度（差速驱动固定为0）
                .z = velocity_z_mrads,  // Z轴角速度（mrad/s）
            },
            .power = 12 * 1000,         // 电池电压（mV），此处为固定值12V
            .checksum = 0,              // 校验和，将在下面计算
            .tail = ROS_TAIL,           // 数据帧尾标识
        };
        
        // 步骤8：计算XOR校验和
        uint8_t *sp = frame.buffer;
        uint8_t *ep = frame.buffer + offsetof(ros_send_data_frame_t, checksum);
        for (uint8_t *p = sp; p < ep; ++p) {
            frame.checksum ^= *p;
        }
        
        // 步骤9：通过UDP发送数据帧到上位机
        if (xSemaphoreTake(g_chassis_send_muetx, 0)) {  // 非阻塞获取发送互斥锁
            sendto(sock, frame.buffer, sizeof(frame), 0, 
                  (struct sockaddr *)&g_chassis_target_addr, sizeof(g_chassis_target_addr));
            xSemaphoreGive(g_chassis_send_muetx);
        }
        
        // 50ms发送周期，对应20Hz数据更新率
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

/**
 * @brief ROS数据接收任务
 * 
 * 负责接收上位机发送的速度控制指令并转换为电机控制信号：
 * - 接收UDP数据包，处理数据帧分片和组装
 * - 数据完整性检查和校验
 * - 逆运动学解算：机器人速度 → 左右轮RPM
 * - 硬件抽象层：处理对向安装轮子的方向补偿
 * - 实时电机速度设置
 * 
 * 数据流程：
 * UDP接收 → 帧检测 → 校验 → 解析速度指令 → 运动学解算 → 电机控制
 * 
 * @param pvParameters UDP套接字文件描述符
 */
static void ros_recv_task(void *pvParameters) {
    int sock = (int)pvParameters;
    uint8_t buffer[sizeof(ros_recv_data_frame_t) * 3];  // 接收缓冲区，可容纳3个数据帧
    size_t len = 0;                                     // 缓冲区当前数据长度

    for (;;) {
        // 步骤1：接收UDP数据包（线程安全）
        xSemaphoreTake(g_chassis_recv_muetx, portMAX_DELAY);
        ssize_t recvlen = recv(sock, buffer + len, sizeof(buffer) - len, 0);
        xSemaphoreGive(g_chassis_recv_muetx);
        
        // 检查接收是否成功
        if (recvlen <= 0) {
            g_ros_ctrl = false;    // 接收失败，标记ROS控制无效
            continue;
        }
        len += recvlen;  // 更新缓冲区数据长度
        // 调试信息（已注释）
        //ESP_LOGI(TAG, "Received %d bytes, total buffer len: %d", recvlen, len);
        
        // 步骤2：数据帧解析和处理
        uint8_t *sp = buffer;        // 搜索起始指针
        uint8_t *ep = buffer + len;  // 缓冲区结束指针
        
        // 在缓冲区中搜索完整的数据帧
        while (ep - sp >= sizeof(ros_recv_data_frame_t)) {
            // 检查当前位置是否为有效数据帧头
            if (!ros_recv_data_format_check(sp)) {
                ++sp;  // 不是有效帧头，向前移动一字节继续搜索
            } else {
                // 步骤3：找到有效数据帧，提取速度控制指令
                //ESP_LOGI(TAG, "接收一帧完整数据校验成功 ");
                vec3_t velocity;
                memcpy(&velocity, sp + offsetof(ros_recv_data_frame_t, velocity), sizeof(velocity));
                //ESP_LOGI(TAG, "get velocity--x:%d, z:%d", velocity.x, velocity.z);
            
                // 步骤4：单位转换 - 从上位机单位转换为标准SI单位
                float linear_x = velocity.x / 1000.0f;    // 线速度：mm/s -> m/s
                float angular_z = velocity.z / 1000.0f;   // 角速度：mrad/s -> rad/s
                
                // 步骤5：机器人物理参数单位转换
                float wheel_space_m = g_ctx->wheel_space / 1000.0f;      // 轮距：mm -> m
                float wheel_perimeter_m = g_ctx->wheel_perimeter / 1000.0f; // 轮子周长：mm -> m
                
                // 步骤6：逆运动学解算 - 从机器人整体运动分解为左右轮速度
                // 差速驱动运动学公式：
                // v_left = v_linear - ω * L/2  (左轮线速度)
                // v_right = v_linear + ω * L/2 (右轮线速度)
                float l_rpm = (linear_x - angular_z * wheel_space_m / 2.0f) / wheel_perimeter_m;
                float r_rpm = (linear_x + angular_z * wheel_space_m / 2.0f) / wheel_perimeter_m;
                
                // 步骤7：单位转换 - 从每秒转数转换为每分钟转数
                l_rpm *= 60.0f;  // rps -> rpm
                r_rpm *= 60.0f;  // rps -> rpm
                
                // 步骤8：硬件抽象层 - 处理对向安装轮子的方向补偿
                mc_set_expect_rpm(g_ctx->mc.left, -l_rpm);   // 左轮对向安装，设置反向RPM
                mc_set_expect_rpm(g_ctx->mc.right, r_rpm);   // 右轮正向安装，直接设置RPM
                
                // 标记ROS控制有效，输出调试信息
                g_ros_ctrl = true;
                ESP_LOGI(TAG, "Get linear: %.3f m/s, angular: %.3f rad/s", linear_x, angular_z);
                ESP_LOGI(TAG, "kinematics: left=%.2f rpm, right=%.2f rpm", l_rpm, r_rpm);
                ESP_LOGI(TAG, "motor_set: left=%.2f rpm, right=%.2f rpm", -l_rpm, r_rpm);
                
                // 移动到下一个数据帧
                sp += sizeof(ros_recv_data_frame_t);
            }
        }
        
        // 步骤9：数据帧处理完毕，整理缓冲区
        len = ep - sp;                    // 计算剩余未处理数据长度
        memmove(buffer, sp, len);         // 将未处理数据移到缓冲区开头，为下次接收做准备
    }

    vTaskDelete(NULL);  // 任务结束（正常情况下不会执行到这里）
}

// ========================= 激光雷达DMA回调函数 =========================

/**
 * @brief UART DMA数据回调函数 - 激光雷达数据实时转发
 * 
 * 在UART DMA接收到激光雷达数据时被调用，实现零拷贝高速转发：
 * - 运行在中断上下文，必须快速处理避免阻塞
 * - 直接从DMA缓冲区转发数据到UDP套接字
 * - 使用ISR安全的信号量操作
 * - 支持FreeRTOS任务调度优化
 * 
 * 性能特点：
 * - 零拷贝转发：直接发送DMA缓冲区数据
 * - 非阻塞操作：使用MSG_DONTWAIT标志
 * - 中断安全：使用FromISR版本的信号量函数
 * 
 * @param port UART端口号
 * @param data 激光雷达原始数据指针
 * @param len 数据长度（字节）
 */
static void lidar_dma_data_callback(uart_port_t port, const uint8_t* data, size_t len) 
{
    // 检查数据有效性和目标连接状态
    if (len > 0 && g_has_lidar_target && g_lidar_sock >= 0) { 
        // ISR上下文中的任务切换控制
        BaseType_t higher_priority_task_woken = pdFALSE;
        
        // 中断安全的信号量获取（非阻塞）
        if (xSemaphoreTakeFromISR(g_lidar_send_muetx, &higher_priority_task_woken)) {
            // 零拷贝UDP发送：直接转发DMA缓冲区数据
            ssize_t sent = sendto(g_lidar_sock, data, len, MSG_DONTWAIT, 
                                (struct sockaddr *)&g_lidar_target_addr, 
                                sizeof(g_lidar_target_addr));
            
            // 中断安全的信号量释放
            xSemaphoreGiveFromISR(g_lidar_send_muetx, &higher_priority_task_woken);
            
            // 如果有高优先级任务被唤醒，请求任务切换
            if (higher_priority_task_woken) {
                portYIELD_FROM_ISR(higher_priority_task_woken);
            }
            
            // 错误统计（避免在ISR中使用复杂日志）
            if (sent < 0) {
                // 发送失败，可以考虑计数或其他轻量级处理
                static uint32_t error_count = 0;
                error_count++;
            }
        }
    }
}

/**
 * @brief UART DMA错误回调函数 - 激光雷达通信错误处理
 * 
 * 当UART通信出现错误时被调用，用于错误诊断和系统稳定性监控：
 * - 记录不同类型的通信错误
 * - 提供故障诊断信息
 * - 帮助优化系统配置
 * 
 * 常见错误类型：
 * - FIFO溢出：数据处理速度跟不上接收速度
 * - 缓冲区满：应用层处理不及时
 * - 奇偶校验错误：信号质量问题
 * - 帧错误：波特率不匹配或硬件问题
 * 
 * @param port UART端口号
 * @param error_type UART错误事件类型
 */
static void lidar_dma_error_callback(uart_port_t port, uart_event_type_t error_type) 
{
    switch (error_type) {
        case UART_FIFO_OVF:
            ESP_LOGW(TAG, "Lidar UART FIFO overflow on port %d - 数据处理跟不上接收速度", port);
            break;
        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "Lidar UART buffer full on port %d - 应用层处理不及时", port);
            break;
        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "Lidar UART parity error on port %d - 奇偶校验失败", port);
            break;
        case UART_FRAME_ERR:
            ESP_LOGW(TAG, "Lidar UART frame error on port %d - 帧格式错误", port);
            break;
        default:
            ESP_LOGD(TAG, "Lidar UART error %d on port %d - 其他UART错误", error_type, port);
            break;
    }
}

/**
 * @brief 激光雷达数据转发任务 - DMA高性能版本
 * 
 * 负责初始化和管理激光雷达的UART DMA接收系统：
 * - 配置UART DMA参数（波特率、缓冲区、回调函数）
 * - 启动DMA事件驱动的数据接收
 * - 监控数据传输统计和错误状态
 * - 提供系统运行状态报告
 * 
 * 工作模式：
 * - 纯接收模式：只接收激光雷达数据，不发送命令
 * - 零拷贝转发：DMA直接将数据转发到UDP套接字
 * - 事件驱动：基于UART硬件事件，不占用CPU轮询时间
 * 
 * 性能优势：
 * - 支持高波特率连续数据流（230400bps典型）
 * - 低延迟：硬件DMA + 中断回调
 * - 低CPU占用：事件驱动，无需轮询
 * 
 * @param pvParameters UDP套接字文件描述符（强制转换为int）
 */
static void lidar_send_task(void *pvParameters) {
    int sock = (int)pvParameters;
    g_lidar_sock = sock;  // 保存socket供DMA回调函数使用
    
    // 获取默认DMA配置
    uart_dma_config_t dma_config = uart_dma_get_default_config(LIDAR_UART_NUM, UART1_RXD_PIN, baud_R);
    
    // 自定义配置
    dma_config.tx_pin = UART_PIN_NO_CHANGE;  // 只接收，不发送
    dma_config.rx_buffer_size = 2048;        // 2KB接收缓冲区
    dma_config.tx_buffer_size = 0;           // 不使用发送缓冲区
    dma_config.data_callback = lidar_dma_data_callback;    // 数据回调
    dma_config.error_callback = lidar_dma_error_callback;  // 错误回调
    
    // 初始化DMA UART
    uart_dma_err_t ret = uart_dma_init(&dma_config);
    if (ret != UART_DMA_OK) {
        ESP_LOGE(TAG, "Failed to initialize lidar UART DMA: %d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    // 启动DMA接收
    ret = uart_dma_start(LIDAR_UART_NUM);
    if (ret != UART_DMA_OK) {
        ESP_LOGE(TAG, "Failed to start lidar UART DMA: %d", ret);
        uart_dma_deinit(LIDAR_UART_NUM);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Lidar DMA task started successfully on UART%d", LIDAR_UART_NUM);
    
    // 任务主循环 - 现在主要用于监控和统计
    uint32_t stats_report_counter = 0;
    uart_dma_stats_t stats;
    
    for (;;) {
        // 定期检查连接状态
        if (!g_has_lidar_target) {
            // 如果没有目标，暂停一段时间
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // 每10秒报告一次统计信息
        if (++stats_report_counter >= 10) {
            stats_report_counter = 0;
            
            if (uart_dma_get_stats(LIDAR_UART_NUM, &stats) == UART_DMA_OK) {
                ESP_LOGI(TAG, "Lidar stats - RX: %lu bytes, %lu packets, Errors: FIFO:%lu Buffer:%lu", 
                         stats.bytes_received, stats.packets_received,
                         stats.fifo_overflows, stats.buffer_overflows);
            }
        }
        
        // 1秒检查周期
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // 清理资源（正常情况下不会到达这里）
    uart_dma_stop(LIDAR_UART_NUM);
    uart_dma_deinit(LIDAR_UART_NUM);
}

// ========================= 连接配置处理函数 =========================

/**
 * @brief JSON配置字符串解析函数
 * 
 * 解析上位机发送的连接配置JSON，提取目标IP地址、端口号和连接类型：
 * - 支持chassis（底盘控制）、lidar（激光雷达）、remote（遥控器）三种连接类型
 * - 验证IP地址格式和端口号范围
 * - 返回对应的套接字文件描述符
 * 
 * JSON格式示例：
 * {"ip": "192.168.1.100", "port": 8082, "type": "chassis"}
 * {"ip": "192.168.1.100", "port": 8083, "type": "lidar"}
 * 
 * @param raw JSON字符串
 * @param ret_addr 输出参数：解析出的IP地址（网络字节序）
 * @param ret_port 输出参数：解析出的端口号（网络字节序）
 * @return int 对应的套接字文件描述符，失败返回-1
 */
static int json_parse(const char *raw, in_addr_t *ret_addr, in_port_t *ret_port) {
    cJSON *root = cJSON_Parse(raw);
    if (!root) return -1;  // JSON解析失败

    int ret = -1;
    cJSON *item;

    // 解析IP地址字段
    if ((item = cJSON_GetObjectItem(root, "ip")) != NULL) {
        if (!cJSON_IsString(item)) goto cleanup;
        
        struct in_addr addr;
        // 将IP地址字符串转换为网络字节序的二进制格式
        if (inet_pton(AF_INET, item->valuestring, &addr) != 1) goto cleanup;
        *ret_addr = addr.s_addr;
    }

    // 解析端口号字段（验证范围0-65535）
    if (!(item = cJSON_GetObjectItem(root, "port")) || 
        !cJSON_IsNumber(item) || 
        (item->valuedouble < 0 || item->valuedouble > 65535)) {
        goto cleanup;
    }
    *ret_port = htons((in_port_t)item->valuedouble);  // 转换为网络字节序

    // 解析连接类型字段（必须是chassis/lidar/remote之一）
    if (!(item = cJSON_GetObjectItem(root, "type")) || 
        !cJSON_IsString(item) ||
        (strcmp(item->valuestring, "chassis") && strcmp(item->valuestring, "lidar") && strcmp(item->valuestring, "remote") )) {
        goto cleanup;
    }

    // 根据连接类型返回对应的套接字文件描述符
    if (strcmp(item->valuestring, "chassis") == 0) {
        ret = g_chassis_sock;      // 底盘控制套接字（UDP 8082）
    } else if (strcmp(item->valuestring, "lidar") == 0) {
        ret = g_lidar_sock;        // 激光雷达数据套接字（UDP 8083）
    } else if (strcmp(item->valuestring, "remote") == 0) {
        ret = g_remote_sock;       // 遥控器数据套接字
    }

cleanup:
    cJSON_Delete(root);  // 释放JSON对象内存
    return ret;
}

/**
 * @brief TCP连接处理任务
 * 
 * 处理单个TCP客户端连接的配置请求：
 * - 接收JSON格式的连接配置
 * - 解析并验证配置参数
 * - 设置对应UDP通道的目标地址
 * - 返回本地端口信息给客户端
 * 
 * 连接配置流程：
 * 1. 上位机通过TCP发送JSON配置
 * 2. ESP32解析配置并设置UDP目标
 * 3. ESP32返回本地UDP端口号
 * 4. 上位机开始UDP数据通信
 * 
 * @param pvParameters TCP客户端套接字文件描述符
 */

static void conn_process_handler(void *pvParameters) {
    char buffer[256];
    int sock = (int)pvParameters;
    struct sockaddr_in addr;
    socklen_t socklen = sizeof(addr);
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
        (struct sockaddr *)&addr, &socklen);
    if (len > 0) {
        const char *type;
        buffer[len] = 0;
        ESP_LOGI(TAG, "Received config: %s", buffer); 
        int my_sock = json_parse(buffer, &addr.sin_addr.s_addr, &addr.sin_port);
        if (my_sock == -1) {
            ESP_LOGW(TAG, "parse json string failed");
            goto err;
        } else if (my_sock == g_chassis_sock) {
            xSemaphoreTake(g_chassis_send_muetx, portMAX_DELAY);
            xSemaphoreTake(g_chassis_recv_muetx, portMAX_DELAY);
            memcpy(&g_chassis_target_addr, &addr, sizeof(addr));
            xSemaphoreGive(g_chassis_recv_muetx);
            xSemaphoreGive(g_chassis_send_muetx);
            type = "chassis";
        } else if (my_sock == g_lidar_sock) {
            xSemaphoreTake(g_lidar_send_muetx, portMAX_DELAY);
            memcpy(&g_lidar_target_addr, &addr, sizeof(addr));
            g_has_lidar_target = true; 
            ESP_LOGI(TAG, "Lidar target address set to %s:%d", 
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
            xSemaphoreGive(g_lidar_send_muetx);
            type = "lidar";
        }
         else {
            xSemaphoreTake(g_remote_muetx, portMAX_DELAY);
            (void)connect(my_sock, (struct sockaddr *)&addr, sizeof(addr));
            xSemaphoreGive(g_remote_muetx);
            type = "remote";
        }
        char ip_str[IP4ADDR_STRLEN_MAX];
        inet_ntop(AF_INET, &addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "%s connect to %s:%"PRIu16, type, ip_str, ntohs(addr.sin_port));
        struct sockaddr_in addr;
        socklen_t socklen = sizeof(addr);
        getsockname(my_sock, (struct sockaddr *)&addr, &socklen);
        size_t len = snprintf(buffer, sizeof(buffer), "{\"port\": %"PRIu16"}", ntohs(addr.sin_port));
        printf("向PC发送端口%s\r\n",buffer);
        send(sock, buffer, len, 0);
    }
err:
    close(sock);
    vTaskDelete(NULL);
}

/**
 * @brief TCP连接管理主任务
 * 
 * 作为TCP服务器运行在8080端口，负责管理上位机的连接配置：
 * - 监听TCP连接请求
 * - 为每个连接创建独立的处理任务
 * - 设置接收超时防止阻塞
 * - 支持多个客户端并发连接
 * 
 * 工作流程：
 * 1. 绑定8080端口并开始监听
 * 2. 接受客户端连接
 * 3. 为每个连接创建conn_process_handler任务
 * 4. 处理完成后自动清理连接资源
 * 
 * 并发处理：
 * - 每个客户端连接都有独立的处理任务
 * - 支持chassis、lidar、remote多种连接类型同时配置
 * - 任务自动清理，无内存泄漏
 * 
 * @param pvParameters TCP服务器套接字文件描述符
 */
static void conn_task(void *pvParameters) {
    int sock = (int)pvParameters;
    listen(sock, 5);  // 监听TCP连接，最大5个待处理连接
    const struct timeval timeout = {.tv_sec = 10, .tv_usec = 0};  // 10秒接收超时

    for (;;) {
        // 阻塞等待客户端连接
        int c = accept(sock, NULL, NULL);
        if (c != -1) {
            // 设置接收超时，防止客户端不发送数据时无限等待
            setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            
            // 为每个连接创建独立的处理任务
            xTaskCreate(conn_process_handler, "conn_process", 4096,
                (void *)c, 4, NULL);
        }
    }

    vTaskDelete(NULL);  // 主循环结束（正常不会到达）
}

// ========================= 资源清理函数 =========================

/**
 * @brief 套接字资源清理函数
 * 
 * 关闭所有已打开的网络套接字并重置套接字数组：
 * - 遍历所有套接字（TCP连接、UDP底盘、UDP雷达等）
 * - 安全关闭每个有效的套接字
 * - 重置套接字文件描述符为-1（无效状态）
 */
static void sockfd_del(void) {
    const int *const end = g_socks + sizeof(g_socks) / sizeof(g_socks[0]);
    for (int *sock = g_socks; sock < end; ++sock) {
        if (*sock != -1) {
            close(*sock);      // 关闭套接字
            *sock = -1;        // 标记为无效
        }
    }
}

/**
 * @brief 互斥锁资源清理函数
 * 
 * 删除所有已创建的FreeRTOS互斥锁信号量：
 * - 遥控器数据互斥锁
 * - 底盘数据发送互斥锁  
 * - 底盘数据接收互斥锁
 * - 激光雷达数据发送互斥锁
 * 
 * 安全删除：检查锁是否存在，删除后置NULL防止重复释放
 */
static void mutex_del(void) {
    if (g_remote_muetx != NULL) {
        vSemaphoreDelete(g_remote_muetx);
        g_remote_muetx = NULL;
    }

    if (g_chassis_send_muetx != NULL) {
        vSemaphoreDelete(g_chassis_send_muetx);
        g_chassis_send_muetx = NULL;
    }
    
    if (g_chassis_recv_muetx != NULL) {
        vSemaphoreDelete(g_chassis_recv_muetx);
        g_chassis_recv_muetx = NULL;
    }

    if (g_lidar_send_muetx != NULL) {
        vSemaphoreDelete(g_lidar_send_muetx);
        g_lidar_send_muetx = NULL;
    }
}

/**
 * @brief 任务资源清理函数
 * 
 * 删除所有已创建的FreeRTOS任务：
 * - TCP连接管理任务
 * - ROS数据发送任务
 * - ROS数据接收任务
 * - 激光雷达转发任务
 * 
 * 安全删除：检查任务句柄有效性，删除后置NULL
 */
static void task_del(void) {
    const TaskHandle_t *const end = g_task_handles + sizeof(g_task_handles) / sizeof(g_task_handles[0]);
    for (TaskHandle_t *handle = g_task_handles; handle < end; ++handle) {
        if (*handle != NULL) {  // 修正：检查任务句柄本身是否为NULL
            vTaskDelete(*handle);
            *handle = NULL;
        }
    }
}

// ========================= 公开API函数 =========================

/**
 * @brief 启动无线通信服务（主要API函数）
 * 
 * 初始化并启动完整的WiFi网络通信系统，包括：
 * 
 * 系统初始化：
 * - 创建线程安全的互斥锁（底盘发送/接收、激光雷达发送）
 * - 配置网络套接字（TCP管理、UDP数据传输）
 * - 绑定网络端口（8080/8082/8083）
 * - 创建并启动多个并发任务
 * 
 * 网络架构：
 * - TCP 8080：连接配置管理（JSON格式配置交换）
 * - UDP 8082：底盘双向数据交换（速度控制↔状态反馈）
 * - UDP 8083：激光雷达单向数据转发（传感器→上位机）
 * 
 * 并发任务：
 * - conn_task：TCP连接管理
 * - ros_send_task：20Hz状态数据发送  
 * - ros_recv_task：速度控制指令接收
 * - lidar_send_task：DMA激光雷达数据转发
 * 
 * @param ctx 全局上下文指针（包含电机、传感器、物理参数）
 * @return esp_err_t ESP_OK成功，其他值表示初始化失败
 */
esp_err_t start_wireless_conn(context_pack_t *ctx)
{
    esp_err_t ret;

    // 保存全局上下文
    g_ctx = ctx;
    
    // 创建线程同步互斥锁
    g_chassis_send_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_chassis_send_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create chassis send mutex failed");
    g_chassis_recv_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_chassis_recv_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create chassis recv mutex failed");
    g_lidar_send_muetx = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(g_lidar_send_muetx != NULL, ESP_ERR_NO_MEM, err, TAG, "create lidar send mutex failed");

    // 网络套接字超时配置（5秒）
    struct timeval timeout = { .tv_sec = 5, .tv_usec = 0 };

    // ========== TCP连接管理套接字（端口8080） ==========
    struct sockaddr_in addr = {
        .sin_family = AF_INET,           // IPv4地址族
        .sin_addr.s_addr = IPADDR_ANY,   // 监听所有网络接口（0.0.0.0）
        .sin_port = htons(8080),         // TCP管理端口
    };
    g_conn_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    ESP_GOTO_ON_FALSE(g_conn_sock != -1, ESP_FAIL, err, TAG, "create TCP connection socket failed");
    ESP_GOTO_ON_FALSE(bind(g_conn_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "TCP connection socket bind failed");

    // ========== UDP底盘数据套接字（端口8082） ==========
    addr.sin_port = htons(8082);
    g_chassis_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_GOTO_ON_FALSE(g_chassis_sock != -1, ESP_FAIL, err, TAG, "create chassis UDP socket failed");
    ESP_GOTO_ON_FALSE(setsockopt(g_chassis_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != -1,
        ESP_FAIL, err, TAG, "set chassis socket timeout failed");
    ESP_GOTO_ON_FALSE(bind(g_chassis_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "chassis socket bind failed");

    // ========== UDP激光雷达数据套接字（端口8083） ==========
    addr.sin_port = htons(8083);
    g_lidar_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_GOTO_ON_FALSE(g_lidar_sock != -1, ESP_FAIL, err, TAG, "create lidar UDP socket failed");
    ESP_GOTO_ON_FALSE(setsockopt(g_lidar_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != -1,
        ESP_FAIL, err, TAG, "set lidar socket timeout failed");
    ESP_GOTO_ON_FALSE(bind(g_lidar_sock, (struct sockaddr *)&addr, sizeof(addr)) != -1,
        ESP_FAIL, err, TAG, "lidar socket bind failed");

    // ========== 创建并启动并发任务（固定到CPU核心1） ==========
    // TCP连接管理任务（高优先级3）
    xTaskCreatePinnedToCore(conn_task, "conn", 4096, (void *)g_conn_sock, 3, &g_task_handles[0], 1);
    ESP_GOTO_ON_FALSE(g_task_handles[0] != NULL, ESP_FAIL, err, TAG, "create conn_task failed");
    
    // ROS状态数据发送任务（中等优先级2）
    xTaskCreatePinnedToCore(ros_send_task, "ros_send", 4096, (void *)g_chassis_sock, 2, &g_task_handles[1], 1);
    ESP_GOTO_ON_FALSE(g_task_handles[1] != NULL, ESP_FAIL, err, TAG, "create ros_send_task failed");
    
    // ROS速度控制接收任务（中等优先级2）
    xTaskCreatePinnedToCore(ros_recv_task, "ros_recv", 4096, (void *)g_chassis_sock, 2, &g_task_handles[2], 1);
    ESP_GOTO_ON_FALSE(g_task_handles[2] != NULL, ESP_FAIL, err, TAG, "create ros_recv_task failed");
    
    // 激光雷达DMA数据转发任务（中等优先级2）
    xTaskCreatePinnedToCore(lidar_send_task, "lidar_send", 4096, (void *)g_lidar_sock, 2, &g_task_handles[3], 1);
    ESP_GOTO_ON_FALSE(g_task_handles[3] != NULL, ESP_FAIL, err, TAG, "create lidar_send_task failed");

    ESP_LOGI(TAG, "Wireless communication system started successfully");
    ESP_LOGI(TAG, "  - TCP management port: 8080");
    ESP_LOGI(TAG, "  - UDP chassis port: 8082"); 
    ESP_LOGI(TAG, "  - UDP lidar port: 8083");
    
    return ESP_OK;

err:
    ESP_LOGE(TAG, "Wireless communication system initialization failed");
    stop_wireless_conn();  // 清理已分配的资源
    return ret;
}

/**
 * @brief 停止无线通信服务并清理所有资源
 * 
 * 按照正确顺序清理所有系统资源：
 * 1. 删除所有运行中的任务
 * 2. 关闭所有网络套接字
 * 3. 删除所有互斥锁信号量
 * 
 * 该函数是stop_wireless_conn()的对应清理函数，
 * 也会在start_wireless_conn()初始化失败时被调用
 * 
 * @return esp_err_t 总是返回ESP_OK
 */
esp_err_t stop_wireless_conn(void) {
    ESP_LOGI(TAG, "Stopping wireless communication system...");
    
    task_del();    // 删除所有任务
    sockfd_del();  // 关闭所有套接字
    mutex_del();   // 删除所有互斥锁
    
    // 重置全局状态
    g_ctx = NULL;
    g_ros_ctrl = false;
    g_has_lidar_target = false;
    
    ESP_LOGI(TAG, "Wireless communication system stopped");
    return ESP_OK;
}
