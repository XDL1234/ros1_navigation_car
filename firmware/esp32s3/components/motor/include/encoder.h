#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#ifdef __cplusplus
exturn "C" {
#endif // __cplusplus

typedef struct {
    gpio_num_t edge_gpio_num; /*!< GPIO number used by the edge signal, input mode with pull up enabled. Set to -1 if unused */
    gpio_num_t level_gpio_num; /*!< GPIO number used by the level signal, input mode with pull up enabled. Set to -1 if unused */
    int round_count; /*!< The count of pulses for one rotation of the motor */
} encoder_config_t;

struct encoder_t;
typedef struct encoder_t *encoder_handle_t;

esp_err_t encoder_timer_loop_create(TickType_t period_tick);
esp_err_t encoder_timer_loop_del(void);
esp_err_t encoder_timer_loop_start(void);
esp_err_t encoder_timer_loop_stop(void);

/**
 * @brief Create a new encoder, and return the handle
 * 
 * @param[in] config Encoder configuration
 * @param[out] ret_handle Returned encoder handle
 * @return Error code
 */
esp_err_t encoder_new(const encoder_config_t *config, encoder_handle_t *ret_handle);

/**
 * @brief Start rpm computing
 * 
 * @param[in] handle Encoder handle created by `encoder_new()`
 * @return Error code
 */
esp_err_t encoder_add_to_timer_loop(encoder_handle_t handle);

/**
 * @brief Stop rpm computing
 * 
 * @param[in] handle Encoder handle created by `encoder_new()`
 * @return Error code
 */
esp_err_t encoder_remove_from_time_loop(encoder_handle_t handle);

/**
 * @brief Delete encoder handle
 * 
 * @param[in] handle Encoder handle created by `encoder_new()`
 * @return Error code
 */
esp_err_t encoder_del(encoder_handle_t handle);

/**
 * @brief Return the rpm
 * 
 * @param handle Encoder handle created by `encoder_new()`
 * @return rpm
 */
float encoder_get_rpm(encoder_handle_t handle);

#ifdef __cplusplus
}
#endif // __cplusplus
