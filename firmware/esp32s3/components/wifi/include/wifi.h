/**
 * @file wifi.h
 * @brief WiFi网络连接管理模块头文件
 * 
 * 定义了ESP32 WiFi网络功能的接口和数据结构，支持：
 * - STA模式：连接到现有WiFi网络
 * - AP模式：创建WiFi热点供其他设备连接
 * - mDNS服务：设备发现和主机名解析
 * - 连接状态监控和自动重连
 * 
 * @version 1.0
 * @date 2024
 */

#ifndef __MWIFI_H__
#define __MWIFI_H__

#include "esp_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"

#include "lwip/ip4_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "nvs.h"

#if __has_include("wifi_local.h")
#include "wifi_local.h"
#endif

// ========================= WiFi配置参数 =========================

#define WIF_MAX_AP_CONNECTION       2               ///< AP模式最大连接设备数
#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef DEFAULT_WIFI_PASSWORD
#define DEFAULT_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif
#define EXAMPLE_ESP_MAXIMUM_RETRY   10              ///< STA模式最大重连次数
#define DEFAULT_WIFI_AP_PSWD        "87654321"      ///< AP模式热点密码
#define DEFAULT_WIFI_AP_SSID_PREFIX "micu_ros"      ///< AP模式热点名称前缀

// ========================= 数据结构定义 =========================

/**
 * @brief WiFi连接状态枚举类型
 * 
 * 描述ESP32 WiFi子系统的当前连接状态
 */
typedef enum
{
    WIFI_STATUS_STA_DISCONECTED = 0,    ///< STA模式未连接或连接丢失
    WIFI_STATUS_STA_CONNECTED = 1,      ///< STA模式已连接并获取IP地址
    WIFI_STATUS_AP_READY,               ///< AP模式就绪，可接受设备连接
} wifi_status_t;

// ========================= 函数声明 =========================

/**
 * @brief 设置WiFi配置参数（预留接口）
 * 
 * 用于动态配置WiFi连接参数，当前未实现
 * 
 * @param wifi_config WiFi配置结构体指针
 * @return true 配置成功
 * @return false 配置失败
 */
bool set_wifi_config(wifi_config_t *wifi_config);

/**
 * @brief 配置并启动WiFi STA（客户端）模式
 * 
 * 连接到指定的WiFi网络作为客户端设备：
 * - 使用默认SSID和密码（可通过实定APP值修改）
 * - 自动重连机制（最多10次重试）
 * - 使用DHCP获取IP地址
 * - 支持事件驱动的异步连接
 * 
 * @return true STA模式配置成功
 * @return false STA模式配置失败
 */
bool wifi_set_sta();
/**
 * @brief WiFi系统完整初始化
 * 
 * 初始化ESP32的所有WiFi相关系统组件：
 * - NVS闪存系统（用于配置存储）
 * - TCP/IP协议栈初始化
 * - 事件循环系统初始化
 * - WiFi模式配置（STA或AP，根据STA_MODE编译选项）
 * - mDNS服务启动（设备发现）
 * 
 * 初始化順序会自动处理依赖关系，确保系统稳定启动
 * 
 * @return true 初始化成功
 * @return false 初始化失败（不会发生，失败时程序会终止）
 */
bool wifi_init(void);

/**
 * @brief 获取WiFi连接状态和IP地址
 * 
 * 查询当前WiFi连接状态并格式化输出IP地址信息：
 * - STA模式连接成功：输出"HOST:192.168.1.100"格式
 * - STA模式连接失败：输出"HOST:lost_connect"
 * - AP模式就绪：输出"HOST:192.168.43.100"格式
 * 
 * 该函数常用于：
 * - OLED显示屏状态显示
 * - Web界面网络信息显示
 * - 系统状态监控和调试
 * 
 * @param ip_address 输出缓冲区，用于存储格式化的IP地址字符串（至少20字节）
 * @return wifi_status_t 当前WiFi连接状态枚举值
 */
wifi_status_t get_wifi_ip(char *ip_address);

/**
 * @brief 启动mDNS（多播DNS）服务
 * 
 * 启动mDNS服务使设备在本地网络中可通过主机名被发现：
 * - 设置设备主机名，支持"hostname.local"格式访问
 * - 配置服务实例名称，用于服务发现
 * - 简化ROS节点和Web界面的网络配置
 * 
 * 例如，设置主机名为"micu-ros-car"后，
 * 其他设备可以通过"micu-ros-car.local"直接访问
 * 
 * @param hostname 设备主机名（不包含".local"后缀）
 * @param instance_name 服务实例名称，用于标识设备类型
 * @return esp_err_t ESP_OK成功，ESP_FAIL失败
 */
esp_err_t start_mdns_service(const char *hostname, const char *instance_name);
#endif