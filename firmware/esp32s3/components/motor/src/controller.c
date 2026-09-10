/**
 * @file controller.c  
 * @brief 电机控制器核心实现模块
 * 
 * 实现了完整的双轮差速驱动控制系统：
 * - 基于PID的闭环转速控制
 * - 支持TB6612FNG和DRV8833两种电机驱动器
 * - 编码器反馈的精确转速计算
 * - FreeRTOS定时器驱动的周期性控制
 * - 链表管理的多电机并发控制
 * 
 * 控制算法：
 * - PID控制器：比例-积分-微分反馈控制
 * - 编码器测速：基于脉冲计数的转速估算
 * - 方向控制：支持正反转和制动
 * - 转速限制：最大转速保护
 * 
 * 硬件支持：
 * - TB6612FNG：PWM+方向控制模式（推荐）
 * - DRV8833：双PWM差分控制模式
 * - 增量式编码器：A/B相脉冲计数
 * 
 * @version 1.0
 * @date 2024
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/list.h"
#include "esp_check.h"
#include "controller.h"

#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)

// ========================= 核心数据结构定义 =========================

/**
 * @brief 电机控制实例结构体
 * 
 * 包含单个电机的完整控制上下文，支持多电机并发控制
 */
typedef struct mc_t {
    bdc_motor_handle_t motor;           ///< 有刷直流电机驱动句柄
    encoder_handle_t encoder;           ///< 增量式编码器句柄  
    pid_ctrl_block_handle_t pid;        ///< PID反馈控制器句柄
    ListItem_t item;                    ///< FreeRTOS链表节点（用于多电机管理）
    float max_rpm;                      ///< 最大转速限制（RPM）
    float expect_rpm;                   ///< 期望目标转速（RPM）
    pid_ctrl_parameter_t pid_params;    ///< PID控制器参数（Kp、Ki、Kd）
#ifdef USE_TB6612_DRIVER
    gpio_num_t dir1_gpio;              ///< TB6612方向控制引脚1 (IN1)
    gpio_num_t dir2_gpio;              ///< TB6612方向控制引脚2 (IN2)  
    bool is_forward;                   ///< 当前旋转方向 (true=正转, false=反转)
#endif
} mc_t;

// ========================= 宏定义和错误处理 =========================

#define TAG "motor_ctrl"  // 日志标签

// 错误检查宏定义，简化状态验证代码
#define RETURN_ON_FALSE(a, msg) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_STATE, TAG, msg)
#define RETURN_ON_INCREATED(msg) RETURN_ON_FALSE(g_timer_handle == NULL, msg)       // 定时器未创建检查
#define RETURN_ON_UNCREATED(msg) RETURN_ON_FALSE(g_timer_handle != NULL, msg)       // 定时器已创建检查  
#define RETURN_ON_INACTIVE(msg) RETURN_ON_FALSE(!xTimerIsTimerActive(g_timer_handle), msg)  // 定时器未激活检查
#define RETURN_ON_UNACTIVE(msg) RETURN_ON_FALSE(xTimerIsTimerActive(g_timer_handle), msg)   // 定时器已激活检查
#define RETURN_ON_LIST_EMPTY(msg) RETURN_ON_FALSE(!listLIST_IS_EMPTY(&g_list), msg)         // 电机列表非空检查
#define RETURN_ON_LIST_NOT_EMPTY(msg) RETURN_ON_FALSE(listLIST_IS_EMPTY(&g_list), msg)      // 电机列表空检查
#define RETURN_ON_ENCODER_IN_LIST(handle, msg) RETURN_ON_FALSE(!listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)      // 电机不在列表检查
#define RETURN_ON_ENCODER_NOT_IN_LIST(handle, msg) RETURN_ON_FALSE(listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)   // 电机在列表检查

// ========================= 全局变量 =========================

static List_t g_list;                           ///< FreeRTOS链表，管理所有电机实例
static TimerHandle_t g_timer_handle = NULL;     ///< FreeRTOS周期定时器句柄，驱动PID控制循环

#ifdef USE_TB6612_DRIVER
// ========================= TB6612驱动器专用函数 =========================

/**
 * @brief TB6612FNG电机驱动器方向控制
 * 
 * TB6612FNG使用两个方向控制引脚(IN1/IN2)配合PWM实现电机控制：
 * - 正转：IN1=HIGH, IN2=LOW, PWM控制转速
 * - 反转：IN1=LOW, IN2=HIGH, PWM控制转速  
 * - 制动：IN1=HIGH, IN2=HIGH, PWM=0
 * - 关闭：IN1=LOW, IN2=LOW, PWM=0
 * 
 * @param handle 电机控制句柄
 * @param forward true=正转, false=反转
 * @return esp_err_t ESP_OK成功，其他值表示GPIO设置失败
 */
static esp_err_t tb6612_set_direction(mc_handle_t handle, bool forward) {
    if (handle->dir1_gpio != -1 && handle->dir2_gpio != -1) {
        if (forward) {
            // 正转: DIR1=1, DIR2=0
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 1), TAG, "set dir1 gpio failed");
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 0), TAG, "set dir2 gpio failed");
        } else {
            // 反转: DIR1=0, DIR2=1
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 0), TAG, "set dir1 gpio failed");
            ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 1), TAG, "set dir2 gpio failed");
        }
        handle->is_forward = forward;
        ESP_LOGD(TAG, "TB6612 direction set to %s", forward ? "forward" : "reverse");
    }
    return ESP_OK;
}

/**
 * @brief TB6612FNG电机刹车控制
 * 
 * 通过设置TB6612的方向控制引脚实现电机刹车：
 * - 刹车状态：IN1=HIGH, IN2=HIGH
 * - 此时电机两端短路，产生反向电动势实现快速制动
 * - 比自然停车更快，适用于紧急停止
 * 
 * 刹车原理：
 * - 电机绕组短路形成闭环
 * - 转子惯性产生的反向电动势被短路电流消耗
 * - 产生制动转矩使电机快速停止
 * 
 * @param handle 电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示GPIO设置失败
 */
static esp_err_t tb6612_brake(mc_handle_t handle) {
    if (handle->dir1_gpio != -1 && handle->dir2_gpio != -1) {
        // 刹车: DIR1=1, DIR2=1
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir1_gpio, 1), TAG, "set dir1 gpio failed");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->dir2_gpio, 1), TAG, "set dir2 gpio failed");
        ESP_LOGD(TAG, "TB6612 brake applied");
    }
    return ESP_OK;
}

/**
 * @brief 初始化TB6612FNG方向控制GPIO
 * 
 * 配置TB6612FNG电机驱动芯片的方向控制引脚：
 * - 设置GPIO为输出模式
 * - 禁用内部上拉/下拉电阻
 * - 初始化GPIO电平为低（停止状态）
 * - 验证GPIO引脚号有效性
 * 
 * GPIO配置：
 * - DIR1 (IN1)：正转控制引脚
 * - DIR2 (IN2)：反转控制引脚
 * - 两个引脚配合PWM信号实现电机的方向和速度控制
 * 
 * @param handle 电机控制句柄
 * @param dir1_gpio 方向控制引脚1 (IN1)
 * @param dir2_gpio 方向控制引脚2 (IN2)
 * @return esp_err_t ESP_OK成功，其他值表示GPIO初始化失败
 */
static esp_err_t tb6612_init_gpio(mc_handle_t handle, gpio_num_t dir1_gpio, gpio_num_t dir2_gpio) {
    handle->dir1_gpio = dir1_gpio;
    handle->dir2_gpio = dir2_gpio;
    handle->is_forward = true;

    // 初始化DIR1 GPIO
    if (dir1_gpio != -1) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir1_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        
        // 检查GPIO是否有效
        if (dir1_gpio >= SOC_GPIO_PIN_COUNT) {
            return ESP_ERR_INVALID_ARG;
        }
        
        esp_err_t ret = gpio_config(&io_conf);

        ESP_LOGI(TAG, "DIR1 GPIO configured successfully");
        ESP_RETURN_ON_ERROR(gpio_set_level(dir1_gpio, 0), TAG, "set dir1 gpio level failed");
    }

    // 初始化DIR2 GPIO
    if (dir2_gpio != -1) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir2_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        
        // 检查GPIO是否有效
        if (dir2_gpio >= SOC_GPIO_PIN_COUNT) {
            return ESP_ERR_INVALID_ARG;
        }
        
        esp_err_t ret = gpio_config(&io_conf);
        
        ESP_LOGI(TAG, "DIR2 GPIO configured successfully");
        ESP_RETURN_ON_ERROR(gpio_set_level(dir2_gpio, 0), TAG, "set dir2 gpio level failed");
    }

    ESP_LOGI(TAG, "TB6612 GPIO initialization completed successfully");
    return ESP_OK;
}
#endif

// ========================= PID控制循环核心函数 =========================

/**
 * @brief PID控制循环定时器回调函数
 * 
 * 这是整个电机控制系统的核心，运行在FreeRTOS定时器上下文中：
 * - 遍历所有已注册的电机实例
 * - 读取编码器反馈获取实际转速
 * - 执行PID控制算法计算输出
 * - 根据驱动器类型设置电机PWM和方向
 * 
 * 控制算法流程：
 * 1. 获取期望转速和实际转速
 * 2. 计算转速误差 (error = expect - actual)
 * 3. PID控制器根据误差计算PWM占空比
 * 4. 根据PWM正负值确定旋转方向
 * 5. 设置电机驱动器的PWM和方向信号
 * 
 * 驱动器兼容性：
 * - TB6612FNG：PWM+方向控制模式，需要额外GPIO控制方向
 * - DRV8833：双PWM差分控制模式，通过BDC驱动库控制
 * 
 * @param xTimer 定时器句柄（未使用）
 */
static void pid_loop_tcb(TimerHandle_t xTimer) {
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;

        // 1. 读取当前转速和计算误差
        float current_rpm = encoder_get_rpm(handle->encoder);
        float error = handle->expect_rpm - current_rpm;

        //ESP_LOGI(TAG, "<rpm>:%.2f,%.2f\n", handle->expect_rpm,current_rpm);
        // 2. PID计算
        float new_duty;
        pid_compute(handle->pid, error, &new_duty);

        // 3. 根据驱动类型执行电机控制
#ifdef USE_TB6612_DRIVER
        if (new_duty > 0) {
            // 正转
            tb6612_set_direction(handle, true);
            bdc_motor_set_speed(handle->motor, new_duty);
        } else if (new_duty < 0) {
            // 反转
            tb6612_set_direction(handle, false);
            bdc_motor_set_speed(handle->motor, -new_duty);
        } else {
            // 停止 (PWM=0)
            bdc_motor_set_speed(handle->motor, 0);
        }
#else
        // DRV8833模式：使用双PWM控制
        if (new_duty > 0) {
            bdc_motor_forward(handle->motor);
            bdc_motor_set_speed(handle->motor, new_duty);
        } else {
            bdc_motor_reverse(handle->motor);
            bdc_motor_set_speed(handle->motor, -new_duty);
        }
#endif

        item = listGET_NEXT(item);
    }
}

/**
 * @brief 启动单个电机实例
 * 
 * 启用电机驱动器并设置为默认前进状态：
 * - 启用BDC电机驱动器
 * - 设置电机为前进方向（默认状态）
 * 
 * 注意：实际运行方向将由PID控制循环根据目标RPM动态调整
 * 
 * @param handle 电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示启动失败
 */
static esp_err_t mc_start(mc_handle_t handle) {
    ESP_LOGD(TAG, "Enable motor");
    ESP_RETURN_ON_ERROR(bdc_motor_enable(handle->motor), TAG, "enable motor failed");
    
    ESP_LOGD(TAG, "Forward motor");
    ESP_RETURN_ON_ERROR(bdc_motor_forward(handle->motor), TAG, "forward motor failed");

    return ESP_OK;
}

/**
 * @brief 停止单个电机实例
 * 
 * 禁用电机驱动器，电机立即停止转动：
 * - 禁用BDC电机驱动器
 * - 电机输出PWM置零
 * - 电机进入空闲状态
 * 
 * @param handle 电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示停止失败
 */
static esp_err_t mc_stop(mc_handle_t handle) {
    ESP_LOGD(TAG, "disable motor unit");
    ESP_RETURN_ON_ERROR(bdc_motor_disable(handle->motor), TAG, "disable motro failed");

    return ESP_OK;
}

// ========================= 定时器系统管理API =========================

/**
 * @brief 创建电机控制定时器循环系统
 * 
 * 初始化整个电机控制系统的定时器基础设施：
 * - 创建编码器读取定时器（用于测速）
 * - 创建PID控制循环定时器（用于速度控制）
 * - 初始化电机实例管理链表
 * 
 * 定时器关系：
 * - 编码器定时器：高频率读取编码器脉冲，计算转速
 * - PID定时器：低频率执行控制算法，调整电机输出
 * - 典型配置：编码器100ms，PID 100ms（10Hz控制频率）
 * 
 * 注意：必须先创建定时器循环，才能创建电机实例
 * 
 * @param pid_period_tick PID控制循环周期（FreeRTOS ticks）
 * @param encoder_period_tick 编码器读取周期（FreeRTOS ticks） 
 * @return esp_err_t ESP_OK成功，其他值表示创建失败
 */
esp_err_t mc_timer_loop_create(TickType_t pid_period_tick, TickType_t encoder_period_tick) {
    RETURN_ON_INCREATED("timer loop has been already created");

    esp_err_t ret;

    ESP_RETURN_ON_ERROR(encoder_timer_loop_create(encoder_period_tick), TAG, "");

    g_timer_handle = xTimerCreate("pid_loop",
        pid_period_tick, pdTRUE, NULL, pid_loop_tcb);
    ESP_GOTO_ON_FALSE(g_timer_handle != NULL, ESP_ERR_NO_MEM, err, TAG, "create timer loop failed");

    vListInitialise(&g_list);

    return ESP_OK;

err:
    ESP_RETURN_ON_ERROR(encoder_timer_loop_del(), TAG, "");
    return ret;
}

/**
 * @brief 删除电机控制定时器循环系统
 * 
 * 清理整个电机控制系统的定时器资源：
 * - 停止并删除PID控制定时器
 * - 删除编码器读取定时器
 * - 检查并确保所有电机实例已从系统中移除
 * 
 * 安全检查：
 * - 验证定时器循环已创建
 * - 确保定时器已停止（非活跃状态）
 * - 确保电机实例链表为空
 * 
 * @return esp_err_t ESP_OK成功，其他值表示删除失败
 */
esp_err_t mc_timer_loop_del(void) {
    RETURN_ON_UNCREATED("timer loop is not created or has been deleted");
    RETURN_ON_INACTIVE("please call `mc_timer_loop_stop` to stop it at first");
    RETURN_ON_LIST_NOT_EMPTY("please delete all encoder handle at first");

    ESP_RETURN_ON_ERROR(encoder_timer_loop_del(), TAG, "");
    ESP_RETURN_ON_FALSE(xTimerDelete(g_timer_handle, 0), ESP_FAIL, TAG, "stop timer loop failed");
    g_timer_handle = NULL;
    return ESP_OK;
}

/**
 * @brief 启动电机控制定时器循环系统
 * 
 * 启动整个电机控制系统的运行：
 * - 启动编码器读取定时器开始测速
 * - 启动所有已注册电机的硬件驱动器
 * - 启动PID控制循环定时器开始速度控制
 * 
 * 启动顺序：
 * 1. 启动编码器定时器（确保有速度反馈）
 * 2. 启动所有电机的BDC驱动器（使能硬件）
 * 3. 启动PID控制定时器（开始闭环控制）
 * 
 * 错误处理：如果启动过程中任何步骤失败，会自动回滚已启动的组件
 * 
 * @return esp_err_t ESP_OK成功，其他值表示启动失败
 */
esp_err_t mc_timer_loop_start(void) {
    RETURN_ON_UNCREATED("please call `mc_timer_loop_init` to create timer loop at first");
    RETURN_ON_INACTIVE("timer loop has been already started");

    ESP_RETURN_ON_ERROR(encoder_timer_loop_start(), TAG, "");

    esp_err_t ret;
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    ListItem_t *_item = item;
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;
        ESP_GOTO_ON_ERROR(mc_start(handle), err, TAG, "start failed");
        item = listGET_NEXT(item);
    }

    ESP_GOTO_ON_FALSE(xTimerStart(g_timer_handle, 0), ESP_FAIL, err, TAG, "start timer loop failed");

    return ESP_OK;

err:
    ESP_RETURN_ON_ERROR(encoder_timer_loop_stop(), TAG, "");
    while (_item != item) {
        mc_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(mc_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    return ret;
}

/**
 * @brief 停止电机控制定时器循环系统
 * 
 * 停止整个电机控制系统的运行：
 * - 停止PID控制循环定时器（停止速度控制）
 * - 停止所有电机的BDC驱动器（禁用硬件输出）
 * - 停止编码器读取定时器（停止测速）
 * 
 * 停止顺序：
 * 1. 停止PID控制定时器（防止新的控制输出）
 * 2. 停止所有电机驱动器（确保电机停转）
 * 3. 停止编码器定时器（节约CPU资源）
 * 
 * 安全检查：
 * - 验证定时器循环已创建且正在运行
 * 
 * @return esp_err_t ESP_OK成功，其他值表示停止失败
 */
esp_err_t mc_timer_loop_stop(void) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_UNACTIVE("timer loop is not started or has been stopped");

    ESP_RETURN_ON_FALSE(xTimerStop(g_timer_handle, 0), ESP_FAIL, TAG,
                        "deactivate timer loop failed");

    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        mc_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(mc_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    ESP_RETURN_ON_ERROR(encoder_timer_loop_stop(), TAG, "");

    return ESP_OK;
}

// ========================= 电机实例管理API =========================

/**
 * @brief 创建新的电机控制实例
 * 
 * 创建并配置一个完整的电机控制实例，包括：
 * - BDC电机驱动器（支持MCPWM PWM输出）
 * - 增量式编码器（脉冲计数测速）
 * - PID控制器（闭环速度控制）
 * - TB6612方向控制GPIO（如果使用TB6612驱动）
 * 
 * 初始化流程：
 * 1. 内存分配和参数验证
 * 2. TB6612 GPIO初始化（如果启用）
 * 3. MCPWM设备创建（PWM信号生成）
 * 4. 编码器创建（速度反馈）
 * 5. PID控制器创建（控制算法）
 * 6. 添加编码器到测速定时器循环
 * 
 * 驱动器支持：
 * - TB6612FNG：PWM+方向控制，需要额外的方向控制GPIO
 * - DRV8833：纯PWM控制，通过BDC库的forward/reverse控制
 * 
 * @param config 电机配置结构体指针
 * @param max_rpm 最大转速限制（RPM）
 * @param ret_handle 返回的电机控制句柄指针
 * @return esp_err_t ESP_OK成功，其他值表示创建失败
 */
esp_err_t mc_new(const mc_config_t *config, float max_rpm, mc_handle_t *ret_handle) {
    esp_err_t ret;
    ESP_LOGI(TAG, "Creating new motor controller with max RPM: %.2f", max_rpm);
    
    ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_NO_MEM,
        TAG, "no mem for motor control handle");
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("can not create encoder when timer loop is active");

    mc_handle_t handle = malloc(sizeof(mc_t));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "no mem for motor control handle");

#ifdef USE_TB6612_DRIVER
    ESP_LOGI(TAG, "Using TB6612 driver mode");    
    // TB6612模式下，先初始化方向控制GPIO
    if (config->tb6612_config.dir1_gpio != -1 && config->tb6612_config.dir2_gpio != -1) {
        ESP_GOTO_ON_ERROR(tb6612_init_gpio(handle,
                                         config->tb6612_config.dir1_gpio,
                                         config->tb6612_config.dir2_gpio),
                         err, TAG, "TB6612 GPIO init failed");
    }
#endif

    // 然后创建MCPWM设备
    ESP_LOGI(TAG, "Creating MCPWM device...");
    ret = bdc_motor_new_mcpwm_device(&config->motor_config, &config->mcpwm_config, &handle->motor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Create MCPWM device failed: %s", esp_err_to_name(ret));
        goto err;
    }
    ESP_LOGI(TAG, "MCPWM device created successfully");

    // 创建编码器
    ESP_GOTO_ON_ERROR(encoder_new(&config->encoder_config, &handle->encoder), err,
        TAG, "create encoder handle failed");
    ESP_LOGI(TAG, "Encoder created successfully");

    // 创建PID控制器
    ESP_LOGI(TAG, "Creating PID controller with P=%.2f, I=%.2f, D=%.2f", 
             config->pid_config.init_param.kp,
             config->pid_config.init_param.ki,
             config->pid_config.init_param.kd);
    ESP_GOTO_ON_ERROR(pid_new_control_block(&config->pid_config, &handle->pid), err,
        TAG, "create pid control block failed");
    ESP_LOGI(TAG, "PID controller created successfully");

    ESP_LOGI(TAG, "Adding encoder to timer loop");
    ESP_RETURN_ON_ERROR(encoder_add_to_timer_loop(handle->encoder), TAG, "add encoder to timer loop failed");

#ifdef USE_TB6612_DRIVER
    // 保存TB6612配置
    handle->dir1_gpio = config->tb6612_config.dir1_gpio;
    handle->dir2_gpio = config->tb6612_config.dir2_gpio;
    handle->is_forward = true;

#endif

    handle->expect_rpm = 0;
    handle->max_rpm = max_rpm;
    vListInitialiseItem(&handle->item);
    handle->item.pvOwner = handle;

    handle->pid_params = config->pid_config.init_param;

    *ret_handle = handle;
    ESP_LOGI(TAG, "Motor controller created successfully");
    return ESP_OK;

err:
    if (handle) {
        free(handle);
    }
    ESP_LOGE(TAG, "Failed to create motor controller: %s", esp_err_to_name(ret));
    return ret;
}

/**
 * @brief 删除电机控制实例
 * 
 * 安全删除电机控制实例并释放所有相关资源：
 * - 从编码器定时器循环中移除
 * - 删除BDC电机驱动器
 * - 删除编码器实例
 * - 删除PID控制器
 * - 释放实例内存
 * 
 * 安全检查：
 * - 确保电机不在定时器循环的活跃列表中
 * - 按正确顺序清理各个组件
 * 
 * @param handle 要删除的电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示删除失败
 */
esp_err_t mc_del(mc_handle_t handle) {
    RETURN_ON_ENCODER_IN_LIST(handle, "please remove encoder from timer loop at first");
    
    ESP_RETURN_ON_ERROR(encoder_remove_from_time_loop(handle->encoder), "TAG", "");

    if (handle->motor) {
        ESP_RETURN_ON_ERROR(bdc_motor_del(handle->motor), TAG, "delete bdc motor failed");
        handle->motor = NULL;
    }

    if (handle->encoder) {
        ESP_RETURN_ON_ERROR(encoder_del(handle->encoder), TAG, "delete encoder failed");
        handle->encoder = NULL;
    }

    if (handle->pid) {
        ESP_RETURN_ON_ERROR(pid_del_control_block(handle->pid), TAG, "delete pid control block failed");
        handle->pid = NULL;
    }

    free(handle);

    return ESP_OK;
}

/**
 * @brief 将电机实例添加到定时器循环
 * 
 * 将电机控制实例加入到PID控制循环的管理列表：
 * - 添加到全局电机实例链表
 * - PID定时器将开始对该电机执行控制算法
 * - 编码器将开始提供速度反馈
 * 
 * 前提条件：
 * - 定时器循环系统必须已创建
 * - 定时器循环必须处于停止状态（不能在运行时添加）
 * - 该电机实例不能已经在链表中
 * 
 * @param handle 要添加的电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示添加失败
 */
esp_err_t mc_add_to_timer_loop(mc_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_IN_LIST(handle, "it is already in list");

    vListInsertEnd(&g_list, &handle->item);

    return ESP_OK;
}

/**
 * @brief 从定时器循环中移除电机实例
 * 
 * 将电机控制实例从PID控制循环的管理列表中移除：
 * - 从全局电机实例链表中移除
 * - PID定时器不再对该电机执行控制
 * - 电机将保持当前状态直到手动操作
 * 
 * 前提条件：
 * - 定时器循环系统必须已创建
 * - 定时器循环必须处于停止状态（不能在运行时移除）
 * - 该电机实例必须已经在链表中
 * 
 * @param handle 要移除的电机控制句柄
 * @return esp_err_t ESP_OK成功，其他值表示移除失败
 */
esp_err_t mc_remove_from_timer_loop(mc_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_NOT_IN_LIST(handle, "it is not in list");

    uxListRemove(&handle->item);

    return ESP_OK;
}

// ========================= 运行时控制API =========================

/**
 * @brief 设置PID控制器参数
 * 
 * 动态调整PID控制器的比例、积分、微分参数：
 * - Kp（比例）：响应速度，值越大响应越快但可能振荡
 * - Ki（积分）：稳态精度，消除稳态误差
 * - Kd（微分）：系统阻尼，减少超调和振荡
 * 
 * 参数生效：立即更新到PID控制器，下个控制周期生效
 * 
 * @param handle 电机控制句柄
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void mc_set_pid_params(mc_handle_t handle, float kp, float ki, float kd) {
    handle->pid_params.kp = kp;
    handle->pid_params.ki = ki;
    handle->pid_params.kd = kd;
    pid_update_parameters(handle->pid, &handle->pid_params);
}
/**
 * @brief 设置电机期望转速
 * 
 * 设置电机目标转速，PID控制器将调整PWM输出使实际转速接近目标：
 * - 正值：正方向旋转（前进）
 * - 负值：反方向旋转（后退）
 * - 零值：停止
 * 
 * 转速限制：自动限制在[-max_rpm, +max_rpm]范围内，防止超速
 * 
 * @param handle 电机控制句柄
 * @param expect_rpm 期望转速（RPM），可为负数
 */
void mc_set_expect_rpm(mc_handle_t handle, float expect_rpm) {
    handle->expect_rpm = expect_rpm > handle->max_rpm ? handle->max_rpm
        : expect_rpm < -handle->max_rpm ? -handle->max_rpm : expect_rpm;
}

// ========================= 状态查询API =========================

/**
 * @brief 获取电机实际转速
 * 
 * 读取编码器反馈的实际转速值：
 * - 基于编码器脉冲计数计算
 * - 单位：RPM（每分钟转数）
 * - 正负值表示旋转方向
 * 
 * @param handle 电机控制句柄
 * @return float 实际转速（RPM）
 */
float mc_get_real_rpm(mc_handle_t handle) {
    return encoder_get_rpm(handle->encoder);
}

/**
 * @brief 获取PID控制器参数
 * 
 * 读取当前的PID控制器参数值
 * 
 * @param handle 电机控制句柄
 * @param kp 比例系数输出指针
 * @param ki 积分系数输出指针
 * @param kd 微分系数输出指针
 */
void mc_get_pid_params(mc_handle_t handle, float *kp, float *ki, float *kd) {
    if (handle && kp && ki && kd) {
        *kp = handle->pid_params.kp;
        *ki = handle->pid_params.ki;
        *kd = handle->pid_params.kd;
    }
}

/**
 * @brief 获取电机期望转速
 * 
 * 读取当前设置的目标转速值
 * 
 * @param handle 电机控制句柄
 * @return float 期望转速（RPM），句柄无效时返回0
 */
float mc_get_expect_rpm(mc_handle_t handle) {
    return handle ? handle->expect_rpm : 0.0f;
}


