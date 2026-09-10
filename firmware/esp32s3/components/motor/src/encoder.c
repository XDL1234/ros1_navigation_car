
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/list.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "encoder.h"

#define TAG "encoder"

#define RETURN_ON_FALSE(a, msg) ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_STATE, TAG, msg)
#define RETURN_ON_INCREATED(msg) RETURN_ON_FALSE(g_timer_handle == NULL, msg)
#define RETURN_ON_UNCREATED(msg) RETURN_ON_FALSE(g_timer_handle != NULL, msg)
#define RETURN_ON_INACTIVE(msg) RETURN_ON_FALSE(!xTimerIsTimerActive(g_timer_handle), msg)
#define RETURN_ON_UNACTIVE(msg) RETURN_ON_FALSE(xTimerIsTimerActive(g_timer_handle), msg)
#define RETURN_ON_LIST_EMPTY(msg) RETURN_ON_FALSE(!listLIST_IS_EMPTY(&g_list), msg)
#define RETURN_ON_LIST_NOT_EMPTY(msg) RETURN_ON_FALSE(listLIST_IS_EMPTY(&g_list), msg)
#define RETURN_ON_ENCODER_IN_LIST(handle, msg) RETURN_ON_FALSE(!listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)
#define RETURN_ON_ENCODER_NOT_IN_LIST(handle, msg) RETURN_ON_FALSE(listIS_CONTAINED_WITHIN(&g_list, &handle->item), msg)

static List_t g_list;
static TimerHandle_t g_timer_handle = NULL;

typedef struct encoder_t {
    pcnt_unit_handle_t unit;    // PCNT硬件单元句柄
    pcnt_channel_handle_t chan; // PCNT通道句柄
    float rpm;                  // 当前转速（RPM）
    int pluse_per_round;       // 每圈脉冲数
    ListItem_t item;           // 链表节点（用于管理多个编码器）
} encoder_t;

static void compute_rpm_tcb(TimerHandle_t xTimer) {
    TickType_t interval = pdTICKS_TO_MS(xTimerGetPeriod(xTimer));// 采样间隔(ms)
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        encoder_handle_t handle = item->pvOwner;
        int count;
        // 读取脉冲计数
        ESP_ERROR_CHECK(pcnt_unit_get_count(handle->unit, &count));
        // 计算RPM：(脉冲数 * 1000ms * 60s) / (每圈脉冲数 * 采样间隔ms)
        handle->rpm = (float)(count * 1000 * 60) / (handle->pluse_per_round * interval);
        // 清零计数器，准备下次采样
        pcnt_unit_clear_count(handle->unit);
        item = listGET_NEXT(item);
    }
}

static esp_err_t encoder_start(encoder_handle_t handle) {
    ESP_LOGD(TAG, "enable pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(handle->unit), TAG, "enable pcnt unit failed");
    ESP_LOGD(TAG, "clear pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(handle->unit), TAG, "clear pcnt unit failed");
    ESP_LOGD(TAG, "start pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(handle->unit), TAG, "start pcnt unit failed");
    return ESP_OK;
}

static esp_err_t encoder_stop(encoder_handle_t handle) {
    ESP_LOGD(TAG, "stop pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_stop(handle->unit), TAG, "stop pcnt unit failed");
    ESP_LOGD(TAG, "disable pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_unit_disable(handle->unit), TAG, "disable pcnt unit failed");
    return ESP_OK;
}

esp_err_t encoder_timer_loop_create(TickType_t period_tick) {
    RETURN_ON_INCREATED("timer loop has been already created");

    g_timer_handle = xTimerCreate("encoder_speed_compute",
        period_tick, pdTRUE, NULL, compute_rpm_tcb);
    ESP_RETURN_ON_FALSE(g_timer_handle != NULL, ESP_ERR_NO_MEM, TAG, "create timer loop failed");

    vListInitialise(&g_list);

    return ESP_OK;
}

esp_err_t encoder_timer_loop_del(void) {
    RETURN_ON_UNCREATED("timer loop is not created or has been deleted");
    RETURN_ON_INACTIVE("please call `encoder_timer_loop_stop` to stop it at first");
    RETURN_ON_LIST_NOT_EMPTY("please delete all encoder handle at first");

    ESP_RETURN_ON_FALSE(xTimerDelete(g_timer_handle, 0), ESP_FAIL, TAG, "stop timer loop failed");
    g_timer_handle = NULL;
    return ESP_OK;
}

esp_err_t encoder_timer_loop_start(void) {
    RETURN_ON_UNCREATED("please call `encoder_timer_loop_init` to create timer loop at first");
    RETURN_ON_INACTIVE("timer loop has been already started");

    esp_err_t ret;
    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    ListItem_t *_item = item;
    while (item != listGET_END_MARKER(&g_list)) {
        encoder_handle_t handle = item->pvOwner;
        ESP_GOTO_ON_ERROR(encoder_start(handle), err, TAG, "start failed");
        item = listGET_NEXT(item);
    }

    ESP_GOTO_ON_FALSE(xTimerStart(g_timer_handle, 0), ESP_FAIL, err, TAG, "start timer loop failed");

    return ESP_OK;

err:
    while (_item != item) {
        encoder_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(encoder_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    return ret;
}

esp_err_t encoder_timer_loop_stop(void) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_UNACTIVE("timer loop is not started or has been stopped");

    ESP_RETURN_ON_FALSE(xTimerStop(g_timer_handle, 0), ESP_FAIL, TAG,
                        "deactivate timer loop failed");

    ListItem_t *item = listGET_HEAD_ENTRY(&g_list);
    while (item != listGET_END_MARKER(&g_list)) {
        encoder_handle_t handle = item->pvOwner;
        ESP_RETURN_ON_ERROR(encoder_stop(handle), TAG, "stop failed");
        item = listGET_NEXT(item);
    }

    return ESP_OK;
}

esp_err_t encoder_new(const encoder_config_t *config, encoder_handle_t *ret_handle) {
    esp_err_t ret;
    ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL && config->round_count > 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("can not create encoder when timer loop is active");

    encoder_handle_t handle = malloc(sizeof(encoder_t));
    ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL && config->round_count > 0,
                        ESP_ERR_NO_MEM, TAG, "no mem for encoder handle");

    handle->pluse_per_round = config->round_count;
    
    // 1. 创建PCNT单元，计数上下限：±32767
    ESP_LOGD(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {.high_limit = SHRT_MAX, .low_limit = SHRT_MIN};
    ESP_GOTO_ON_ERROR(pcnt_new_unit(&unit_config, &handle->unit), err, TAG,
                      "pcnt unit create failed");

    // 2. 设置毛刺滤波器，过滤1000ns以下的毛刺
    ESP_LOGD(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};
    ESP_GOTO_ON_ERROR(pcnt_unit_set_glitch_filter(handle->unit, &filter_config), err, TAG,
                      "pcnt unit set glitch filter failed");

    // 3. 创建PCNT通道，设置边沿和电平动作
    ESP_LOGD(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_config = {0};
    chan_config.edge_gpio_num = config->edge_gpio_num;
    chan_config.level_gpio_num = config->level_gpio_num;
    ESP_GOTO_ON_ERROR(pcnt_new_channel(handle->unit, &chan_config, &handle->chan), err, TAG,
                      "pcnt channel create failed");

    ESP_LOGD(TAG, "set edge and level actions for pcnt channels");
    ESP_GOTO_ON_ERROR(pcnt_channel_set_edge_action(handle->chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                      err, TAG, "pcnt channel set edge action failed");
    ESP_GOTO_ON_ERROR(pcnt_channel_set_level_action(handle->chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                      err, TAG, "pcnt channel set level action failed");

    handle->rpm = 0;
    vListInitialiseItem(&handle->item);
    handle->item.pvOwner = handle;

    *ret_handle = handle;
    return ESP_OK;

err:
    encoder_del(handle);
    return ret;
}

esp_err_t encoder_del(encoder_handle_t handle) {
    RETURN_ON_ENCODER_IN_LIST(handle, "please remove encoder from timer loop at first");

    if (handle->chan != NULL) {
        ESP_RETURN_ON_ERROR(pcnt_del_channel(handle->chan), TAG, "delete pcnt channel failed");
    }

    if (handle->unit != NULL) {
        ESP_RETURN_ON_ERROR(pcnt_del_unit(handle->unit), TAG, "delete pcnt unit failed");
    }

    free(handle);

    return ESP_OK;
}

esp_err_t encoder_add_to_timer_loop(encoder_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_IN_LIST(handle, "it is already in list");

    vListInsertEnd(&g_list, &handle->item);

    return ESP_OK;
}

esp_err_t encoder_remove_from_time_loop(encoder_handle_t handle) {
    RETURN_ON_UNCREATED("timer loop is not created");
    RETURN_ON_INACTIVE("the function can be call if and only if timer loop is not active");
    RETURN_ON_ENCODER_NOT_IN_LIST(handle, "it is not in list");

    uxListRemove(&handle->item);

    return ESP_OK;
}

float encoder_get_rpm(encoder_handle_t handle) { return handle->rpm; }
