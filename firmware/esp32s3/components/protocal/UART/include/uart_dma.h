/**
 * @file uart_dma.h
 * @brief UART DMA高性能串口通信模块头文件
 * 
 * 提供基于ESP32 UART DMA的高性能串口通信接口：
 * - 事件驱动的非阻塞数据接收
 * - 支持多UART端口并发操作  
 * - 自动错误检测与恢复
 * - 实时统计信息监控
 * - 用户自定义数据和错误回调
 * 
 * 典型使用流程：
 * 1. uart_dma_get_default_config() - 获取默认配置
 * 2. 设置回调函数和特定参数
 * 3. uart_dma_init() - 初始化DMA实例
 * 4. uart_dma_start() - 启动数据接收
 * 5. 在回调中处理接收到的数据
 * 6. uart_dma_stop() / uart_dma_deinit() - 停止和清理
 * 
 * @version 1.0
 * @date 2024
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认配置参数
#define UART_DMA_DEFAULT_BUF_SIZE     (2048)      // 默认缓冲区大小
#define UART_DMA_DEFAULT_TIMEOUT_MS   (100)       // 默认接收超时

// DMA配置结构
typedef struct {
    uart_port_t uart_port;          // UART端口号
    int tx_pin;                     // TX引脚（如果不使用设为UART_PIN_NO_CHANGE）
    int rx_pin;                     // RX引脚
    int baudrate;                   // 波特率
    uart_word_length_t data_bits;   // 数据位（默认8位）
    uart_parity_t parity;           // 校验位（默认无校验）
    uart_stop_bits_t stop_bits;     // 停止位（默认1位）
    uart_hw_flowcontrol_t flow_ctrl; // 流控制（默认关闭）
    size_t rx_buffer_size;          // 接收缓冲区大小
    size_t tx_buffer_size;          // 发送缓冲区大小
    size_t event_queue_size;        // 事件队列大小
    int intr_alloc_flags;           // 中断分配标志
    QueueHandle_t uart_queue;       // UART事件队列句柄（输出）
    TaskHandle_t rx_task_handle;    // 接收任务句柄（输出）
    void (*data_callback)(uart_port_t port, const uint8_t* data, size_t len); // 数据回调函数
    void (*error_callback)(uart_port_t port, uart_event_type_t error_type);   // 错误回调函数
} uart_dma_config_t;

// 错误码
typedef enum {
    UART_DMA_OK = 0,
    UART_DMA_INIT_FAIL,
    UART_DMA_TASK_CREATE_FAIL,
    UART_DMA_INVALID_PARAM,
    UART_DMA_ALREADY_INIT,
    UART_DMA_NOT_INIT,
    UART_DMA_START_FAIL,
    UART_DMA_STOP_FAIL
} uart_dma_err_t;

// 统计信息结构
typedef struct {
    uint32_t bytes_received;        // 接收字节数
    uint32_t packets_received;      // 接收包数
    uint32_t callback_calls;        // 回调调用次数
    uint32_t fifo_overflows;        // FIFO溢出次数
    uint32_t buffer_overflows;      // 缓冲区溢出次数
    uint32_t parity_errors;         // 校验错误次数
    uint32_t frame_errors;          // 帧错误次数
    uint32_t break_errors;          // 中断错误次数
} uart_dma_stats_t;

/**
 * @brief 获取默认DMA配置
 * 
 * @param uart_port UART端口号
 * @param rx_pin 接收引脚
 * @param baudrate 波特率
 * @return uart_dma_config_t 默认配置结构
 */
uart_dma_config_t uart_dma_get_default_config(uart_port_t uart_port, int rx_pin, int baudrate);

/**
 * @brief 初始化UART DMA
 * 
 * @param config DMA配置参数
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_init(const uart_dma_config_t* config);

/**
 * @brief 启动UART DMA接收
 * 
 * @param uart_port UART端口号
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_start(uart_port_t uart_port);

/**
 * @brief 停止UART DMA接收
 * 
 * @param uart_port UART端口号
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_stop(uart_port_t uart_port);

/**
 * @brief 反初始化UART DMA
 * 
 * @param uart_port UART端口号
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_deinit(uart_port_t uart_port);

/**
 * @brief 获取UART DMA统计信息
 * 
 * @param uart_port UART端口号
 * @param stats 统计信息结构指针
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_get_stats(uart_port_t uart_port, uart_dma_stats_t* stats);

/**
 * @brief 重置UART DMA统计信息
 * 
 * @param uart_port UART端口号
 * @return uart_dma_err_t 错误码
 */
uart_dma_err_t uart_dma_reset_stats(uart_port_t uart_port);

/**
 * @brief 发送数据（如果配置了TX）
 * 
 * @param uart_port UART端口号
 * @param data 数据指针
 * @param len 数据长度
 * @param timeout_ms 超时时间（毫秒）
 * @return int 实际发送的字节数，负数表示错误
 */
int uart_dma_send(uart_port_t uart_port, const uint8_t* data, size_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // UART_DMA_H