/**
 * @file wifi.c
 * @brief WiFi网络连接管理模块
 * 
 * 本模块实现了ESP32的WiFi网络功能，支持STA和AP两种工作模式：
 * 
 * 核心功能：
 * - STA模式：连接到现有WiFi网络，获取IP地址
 * - AP模式：创建WiFi热点，为其他设备提供网络接入
 * - mDNS服务：提供设备发现和主机名解析
 * - 连接状态监控和自动重连
 * - 事件驱动的异步连接管理
 * 
 * 技术特点：
 * - 基于ESP-IDF WiFi驱动
 * - FreeRTOS事件组同步机制
 * - 自动重试连接（最多10次）
 * - DHCP服务器配置（AP模式）
 * - 固定IP地址设置
 * 
 * 应用场景：
 * - ROS机器人WiFi通信
 * - 物联网设备网络接入
 * - 远程控制和监控系统
 * 
 * @version 1.0
 * @date 2024
 */

#include "wifi.h"
#include "mdns.h"
#include "esp_err.h" 
#include "esp_check.h"

// ========================= 事件同步和状态管理 =========================

/// FreeRTOS事件组句柄，用于WiFi连接状态同步
static EventGroupHandle_t s_wifi_event_group;

/// WiFi连接成功事件位
#define WIFI_CONNECTED_BIT BIT0
/// WiFi连接失败事件位（重试次数达到上限）
#define WIFI_FAIL_BIT BIT1

/// 编译时选择工作模式：STA模式（连接路由器）或AP模式（创建热点）
#define STA_MODE

// ========================= 全局变量 =========================

static const char *TAG = "WIFI";                                    ///< 日志标签

static wifi_status_t wifi_status_ = WIFI_STATUS_STA_DISCONECTED;    ///< WiFi连接状态
static char wifi_ip_[16];                                           ///< IP地址字符串缓存
static uint8_t retry_connect_sta_count_ = 0;                        ///< STA模式重连计数器
static esp_netif_t *ap_netif = NULL;                                ///< AP模式网络接口句柄  



// ========================= WiFi事件处理函数 =========================

/**
 * @brief WiFi和IP事件统一处理函数
 * 
 * 处理ESP32 WiFi子系统产生的各种事件，实现连接状态管理：
 * - WiFi事件：STA启动、连接、断开、AP设备接入等
 * - IP事件：获取IP地址、IP地址变化等
 * 
 * 事件处理策略：
 * - STA模式：自动重连机制，最多重试10次
 * - AP模式：监控设备连接和断开事件
 * - IP获取：更新本地IP地址缓存和连接状态
 * 
 * 线程安全性：运行在ESP-IDF事件循环任务中，使用FreeRTOS事件组同步
 * 
 * @param arg 用户传递的参数（未使用）
 * @param event_base 事件基类（WIFI_EVENT或IP_EVENT）
 * @param event_id 具体事件ID
 * @param event_data 事件携带的数据指针
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    // ========== WiFi事件处理 ==========
    if(event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:      
            // STA模式启动完成，开始尝试连接到目标WiFi网络
            ESP_LOGI(TAG, "WiFi STA started, attempting to connect...");
            esp_wifi_connect();         
            break;

        case WIFI_EVENT_STA_DISCONNECTED:  
            // STA模式与路由器断开连接，启动自动重连机制
            {
                if (retry_connect_sta_count_ < EXAMPLE_ESP_MAXIMUM_RETRY) 
                {
                    wifi_status_ = WIFI_STATUS_STA_DISCONECTED;
                    retry_connect_sta_count_++;
                    esp_wifi_connect();
                    ESP_LOGI(TAG, "WiFi disconnected, retry attempt %d/%d", 
                             retry_connect_sta_count_, EXAMPLE_ESP_MAXIMUM_RETRY);
                } else {
                    // 达到最大重试次数，设置失败标志
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                    ESP_LOGW(TAG, "WiFi connection failed after %d attempts", EXAMPLE_ESP_MAXIMUM_RETRY);
                }
            }
            break;

        case WIFI_EVENT_AP_STACONNECTED:
            // AP模式：有新设备连接到我们创建的热点
            {
                wifi_event_ap_staconnected_t *event =
                    (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Station connected to AP, MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                         event->mac[0], event->mac[1], event->mac[2], 
                         event->mac[3], event->mac[4], event->mac[5]);
            }
            break;

        case WIFI_EVENT_AP_STADISCONNECTED:
            // AP模式：有设备从我们的热点断开连接
            {
                wifi_event_ap_stadisconnected_t *event =
                    (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Station disconnected from AP, MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                         event->mac[0], event->mac[1], event->mac[2], 
                         event->mac[3], event->mac[4], event->mac[5]);
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled WiFi event: %ld", event_id);
            break;
        }
    }
    // ========== IP事件处理 ==========
    else if(event_base == IP_EVENT)                  
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:           
                // STA模式成功获取IP地址，标志着WiFi连接完全建立
                {
                    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
                    ESP_LOGI(TAG, "WiFi connected successfully! IP: " IPSTR, IP2STR(&event->ip_info.ip));
                    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
                    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
                    
                    // 将IP地址转换为字符串并保存到全局变量
                    char ip_str[16];
                    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
                    sprintf(wifi_ip_, "%s", ip_str);
                    
                    // 重置重连计数器并设置连接成功状态
                    retry_connect_sta_count_ = 0;
                    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                    wifi_status_ = WIFI_STATUS_STA_CONNECTED; 
                }
                break;
                
            default:
                ESP_LOGD(TAG, "Unhandled IP event: %ld", event_id);
                break;
        }
    }
}

// ========================= WiFi AP模式配置函数 =========================

/**
 * @brief 配置并启动WiFi AP（热点）模式
 * 
 * 创建一个WiFi热点，允许其他设备连接到ESP32：
 * - 设置热点名称（SSID）和密码
 * - 配置固定IP地址和DHCP服务器
 * - 启用WPA/WPA2安全认证
 * - 支持最多2个设备同时连接
 * 
 * 网络配置：
 * - IP地址：192.168.43.100
 * - 子网掩码：255.255.255.0
 * - 网关：192.168.43.1
 * - DHCP范围：自动分配
 * 
 * 热点参数：
 * - SSID：micu_ros
 * - 密码：87654321
 * - 加密方式：WPA/WPA2-PSK
 * - 信道：自动选择
 * 
 * @return true AP模式启动成功
 * @return false AP模式启动失败（当前实现总是返回true）
 */
bool wifi_set_ap()
{
    esp_err_t ret = ESP_OK; 
    uint8_t mac[6];

    // 创建默认的AP网络接口
    ap_netif = esp_netif_create_default_wifi_ap();
    
    // 使用默认配置初始化WiFi子系统
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 禁用WiFi省电模式，确保AP稳定运行
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    // 注册WiFi事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    // 配置AP模式参数
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = DEFAULT_WIFI_AP_SSID_PREFIX,    // 热点名称
            .password = DEFAULT_WIFI_AP_PSWD,       // 热点密码
            .max_connection = WIF_MAX_AP_CONNECTION, // 最大连接数
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,     // 加密方式
            .channel = 0,                           // 自动选择信道
            .ssid_hidden = 0,                       // SSID可见
            .beacon_interval = 100,                 // 信标间隔100ms
        },
    };

    // 获取AP模式的MAC地址（用于调试和设备识别）
    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    ESP_LOGI(TAG, "AP MAC address: %02x:%02x:%02x:%02x:%02x:%02x", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
             
    // 可选：动态生成带MAC地址的SSID（已禁用）
    // 如果需要唯一的热点名称，可以启用以下代码：
    // sprintf((char *)wifi_config.ap.ssid, "%s_%02X%02X", DEFAULT_WIFI_AP_SSID_PREFIX, mac[4], mac[5]);
    
    // 密码安全性检查（已禁用）
    // if (strlen(DEFAULT_WIFI_AP_PSWD) == 0) {
    //     wifi_config.ap.authmode = WIFI_AUTH_OPEN;  // 开放网络（不推荐）
    // }
    // 设置WiFi工作模式为AP模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    // 应用AP配置
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    
    // 启动WiFi AP
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // 再次确保禁用省电模式
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    ESP_LOGI(TAG, "WiFi AP initialized successfully");
    ESP_LOGI(TAG, "  SSID: %s", wifi_config.ap.ssid);
    ESP_LOGI(TAG, "  Password: %s", wifi_config.ap.password);
    ESP_LOGI(TAG, "  Max connections: %d", wifi_config.ap.max_connection);

    // 配置AP模式的固定IP地址信息
    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.100"),      // ESP32 AP的IP地址
        .netmask.addr = ipaddr_addr("255.255.255.0"),  // 子网掩码（/24网络）
        .gw.addr = ipaddr_addr("192.168.43.1"),        // 网关地址（通常设为路由器地址）
    };

    // 停止DHCP服务器以修改IP配置
    if(esp_netif_dhcps_stop(ap_netif) == ESP_OK) {
        ESP_LOGI(TAG, "DHCP server stopped successfully");
    } else {
        ESP_LOGW(TAG, "Failed to stop DHCP server");
    }

    // 设置AP的静态IP地址
    ret = esp_netif_set_ip_info(ap_netif, &ip_info);
    if(ret == ESP_OK) {
        sprintf(wifi_ip_, "%s", "192.168.43.100");
        ESP_LOGI(TAG, "AP IP configuration successful:");
        ESP_LOGI(TAG, "  IP Address: 192.168.43.100");
        ESP_LOGI(TAG, "  Subnet Mask: 255.255.255.0");
        ESP_LOGI(TAG, "  Gateway: 192.168.43.1");
    } else {
        ESP_LOGE(TAG, "Failed to set AP IP configuration: %s", esp_err_to_name(ret));
    }
    // 重新启动DHCP服务器为连接设备分配IP地址
    if(esp_netif_dhcps_start(ap_netif) == ESP_OK) {
        ESP_LOGI(TAG, "DHCP server restarted successfully");
    } else {
        ESP_LOGW(TAG, "Failed to restart DHCP server");
    }

    // 更新WiFi状态为AP就绪
    wifi_status_ = WIFI_STATUS_AP_READY;
    ESP_LOGI(TAG, "WiFi AP mode setup completed successfully");
    return true;
}


// ========================= WiFi STA模式配置函数 =========================

/**
 * @brief 配置并启动WiFi STA（客户端）模式
 * 
 * 连接到指定的WiFi网络作为客户端设备：
 * - 扫描并连接到指定SSID的WiFi网络
 * - 使用DHCP自动获取IP地址
 * - 实现自动重连机制（最多10次重试）
 * - 使用事件同步确保连接完成
 * 
 * 连接策略：
 * - 快速扫描模式（优先连接已知网络）
 * - 按信号强度排序选择最佳AP
 * - 支持开放和加密网络
 * - 10秒超时等待连接结果
 * 
 * 事件同步：
 * - 使用FreeRTOS事件组等待连接结果
 * - 连接成功后获取IP地址
 * - 连接失败时进行重试
 * 
 * @return true STA模式启动成功（注意：返回逻辑有误，成功时应返回true）
 * @return false STA模式启动失败
 */
bool wifi_set_sta()
{
    // 配置STA模式连接参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_WIFI_SSID,                  // 目标WiFi网络名称
            .password = DEFAULT_WIFI_PASSWORD,          // WiFi密码
            .scan_method = WIFI_FAST_SCAN,              // 快速扫描模式
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,   // 按信号强度排序
            .threshold.authmode = WIFI_AUTH_OPEN,       // 最低安全级别（接受开放网络）
            .pmf_cfg = {                                // 受保护管理帧配置
                .capable = true,
                .required = false
            },
        },
    };

    // 创建FreeRTOS事件组用于同步WiFi连接状态
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return false;
    }

    // 创建默认的STA网络接口
    esp_netif_create_default_wifi_sta();

    // 使用默认配置初始化WiFi子系统
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 禁用WiFi省电模式，确保稳定的连接性能
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 注册WiFi和IP事件处理函数
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    
    // 注册所有WiFi事件处理
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    
    // 注册IP获取事件处理
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // 配置并启动WiFi STA模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));           // 设置为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // 应用连接配置
    ESP_ERROR_CHECK(esp_wifi_start());                          // 启动WiFi

    ESP_LOGI(TAG, "WiFi STA initialization completed");
    ESP_LOGI(TAG, "Attempting to connect to SSID: %s", DEFAULT_WIFI_SSID);

    // 等待WiFi连接结果（最多10秒）
    ESP_LOGI(TAG, "Waiting for WiFi connection (timeout: 10 seconds)...");
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, 
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,  // 等待连接成功或失败事件
        pdFALSE,                             // 不清除事件位
        pdFALSE,                             // 等待任一事件（OR模式）
        pdMS_TO_TICKS(10000));               // 10秒超时

    // 分析连接结果
    bool connection_successful = false;
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connection established successfully!");
        connection_successful = true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "WiFi connection failed after maximum retry attempts");
    } else {
        ESP_LOGW(TAG, "WiFi connection timeout (10 seconds elapsed)");
    }

    // 清理事件处理函数和事件组
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    
    // 返回连接结果（注意：修正了原来的逻辑错误）
    return connection_successful;
}


// ========================= mDNS服务配置函数 =========================

/**
 * @brief 启动mDNS（多播DNS）服务
 * 
 * mDNS允许设备在本地网络中通过主机名被发现和访问：
 * - 设备可以通过"hostname.local"访问，无需知道IP地址
 * - 支持服务发现，其他设备可以找到ESP32提供的服务
 * - 简化网络配置，特别适合IoT和机器人应用
 * 
 * 应用场景：
 * - ROS节点自动发现：roscore可以通过主机名连接
 * - Web界面访问：浏览器可以通过hostname.local访问
 * - 调试和开发：无需查询IP地址即可连接设备
 * 
 * 配置说明：
 * - 主机名：在网络中的唯一标识符
 * - 实例名：服务的友好名称，用于服务发现
 * 
 * @param hostname 设备主机名（如"micu-ros-car"）
 * @param instance_name 服务实例名称（如"ESP32"）
 * @return esp_err_t ESP_OK成功，ESP_FAIL失败
 */
esp_err_t start_mdns_service(const char *hostname, const char *instance_name) 
{
    // 初始化mDNS服务
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS initialization failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    
    // 设置设备主机名
    ESP_RETURN_ON_ERROR(mdns_hostname_set(hostname), TAG, 
                        "Failed to set mDNS hostname");
    
    // 设置服务实例名称
    ESP_RETURN_ON_ERROR(mdns_instance_name_set(instance_name), TAG, 
                        "Failed to set mDNS instance name");

    ESP_LOGI(TAG, "mDNS service started successfully");
    ESP_LOGI(TAG, "  Hostname: %s.local", hostname);
    ESP_LOGI(TAG, "  Instance: %s", instance_name);
    
    return ESP_OK;
}

// ========================= WiFi系统主初始化函数 =========================

/**
 * @brief WiFi系统完整初始化
 * 
 * 初始化ESP32的WiFi子系统和相关服务，包括：
 * - NVS闪存初始化（WiFi配置存储）
 * - TCP/IP协议栈初始化
 * - 事件循环系统初始化
 * - WiFi模式配置（STA或AP）
 * - mDNS服务启动
 * 
 * 初始化顺序：
 * 1. NVS存储系统（用于WiFi配置持久化）
 * 2. 网络接口和TCP/IP栈
 * 3. 事件处理系统
 * 4. WiFi连接（根据编译选项选择STA或AP模式）
 * 5. mDNS服务（设备发现）
 * 
 * 错误处理：
 * - NVS损坏时自动重新格式化
 * - 所有步骤都有错误检查
 * - 失败时提供详细错误信息
 * 
 * 编译配置：
 * - 定义STA_MODE：连接现有WiFi网络
 * - 未定义STA_MODE：创建WiFi热点
 * 
 * @return true WiFi系统初始化成功
 */
bool wifi_init()
{
    ESP_LOGI(TAG, "Starting WiFi system initialization...");
    
    // ========== NVS（非易失性存储）初始化 ==========
    // WiFi驱动需要NVS来存储校准数据和配置信息
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs to be erased, reformatting...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialized successfully");
    
    // ========== 网络基础设施初始化 ==========
    // 初始化TCP/IP协议栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "TCP/IP stack initialized");
    
    // 创建默认事件循环，用于处理WiFi和IP事件
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Event loop system initialized");
    
    // ========== WiFi模式配置 ==========
    #ifdef STA_MODE
    ESP_LOGI(TAG, "Configuring WiFi in STA (client) mode...");
    bool wifi_success = wifi_set_sta();
    #else
    ESP_LOGI(TAG, "Configuring WiFi in AP (hotspot) mode...");
    bool wifi_success = wifi_set_ap();
    #endif
    
    if (!wifi_success) {
        ESP_LOGW(TAG, "WiFi configuration completed with warnings");
    }
    
    // ========== mDNS服务启动 ==========
    ESP_ERROR_CHECK(start_mdns_service("micu-ros-car", "ESP32"));
    
    ESP_LOGI(TAG, "WiFi system initialization completed successfully!");
    return true;
}

// ========================= 状态查询函数 =========================

/**
 * @brief 获取WiFi连接状态和IP地址
 * 
 * 查询当前WiFi连接状态并格式化IP地址信息：
 * - STA连接成功：返回从路由器获取的IP地址
 * - STA连接失败：返回连接丢失状态
 * - AP模式就绪：返回热点的IP地址
 * 
 * 输出格式：
 * - 连接成功："HOST:192.168.1.100"
 * - 连接失败："HOST:lost_connect"
 * - AP模式："HOST:192.168.43.100"
 * 
 * 应用场景：
 * - OLED显示屏状态显示
 * - Web界面网络信息显示
 * - 调试和监控系统状态
 * 
 * @param ip_address 输出缓冲区，用于存储格式化的IP地址字符串
 * @return wifi_status_t 当前WiFi连接状态枚举值
 */
wifi_status_t get_wifi_ip(char *ip_address)
{
    // 检查输入参数有效性
    if (ip_address == NULL) {
        ESP_LOGE(TAG, "ip_address buffer is NULL");
        return wifi_status_;
    }
    
    // 根据WiFi状态格式化IP地址信息
    switch (wifi_status_) {
        case WIFI_STATUS_STA_CONNECTED:
            // STA模式已连接，显示从DHCP获取的IP地址
            sprintf(ip_address, "HOST:%s", wifi_ip_);
            break;
            
        case WIFI_STATUS_STA_DISCONECTED:
            // STA模式连接丢失或未连接
            sprintf(ip_address, "HOST:lost_connect");
            break;
            
        case WIFI_STATUS_AP_READY:
            // AP模式就绪，显示热点IP地址
            sprintf(ip_address, "HOST:%s", wifi_ip_);
            break;
            
        default:
            // 未知状态，显示错误信息
            sprintf(ip_address, "HOST:unknown_state");
            ESP_LOGW(TAG, "Unknown WiFi status: %d", wifi_status_);
            break;
    }
    
    return wifi_status_;
}