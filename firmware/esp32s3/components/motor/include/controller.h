#pragma once

#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "encoder.h"
#include "driver/gpio.h"

#ifdef __cplusplus
exturn "C" {
#endif // __cplusplus

// PWM电机时钟频率, 10MHz, 1 tick = 0.1us 
// PWM motor clock frequency, 10MHz, 1 tick = 0.1us
#define PWM_MOTOR_TIMER_RESOLUTION_HZ    10000000

// PWM电机控制频率, 25KHz 
// PWM motor control frequency, 25KHz
#define PWM_MOTOR_FREQ_HZ                25000

// PWM理论最大值(400) 
// PWM Theoretical maximum (400)
#define PWM_MOTOR_DUTY_TICK_MAX          (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)

// 电机死区过滤 
// Motor dead zone filtering
#define PWM_MOTOR_DEAD_ZONE              (200)

// 电机PWM输入最大值 
// Maximum motor PWM input value
#define PWM_MOTOR_MAX_VALUE              (PWM_MOTOR_DUTY_TICK_MAX-PWM_MOTOR_DEAD_ZONE)

// 取消注释下面的行来启用TB6612模式，否则使用DRV8833模式
#define USE_TB6612_DRIVER

#ifdef USE_TB6612_DRIVER
    #define MOTOR_DRIVER_TYPE "TB6612"
#else
    #define MOTOR_DRIVER_TYPE "DRV8833"
#endif

#ifdef USE_TB6612_DRIVER
// TB6612驱动配置结构体
typedef struct {
    gpio_num_t pwm_gpio;    /*!< PWM信号GPIO */
    gpio_num_t dir1_gpio;   /*!< 方向控制1 GPIO (IN1) */
    gpio_num_t dir2_gpio;   /*!< 方向控制2 GPIO (IN2) */
} tb6612_config_t;
#endif


// 电机控制配置结构体
typedef struct {
    bdc_motor_config_t motor_config;        /*!< 电机驱动配置 */
    bdc_motor_mcpwm_config_t mcpwm_config;  /*!< MCPWM配置 */
    encoder_config_t encoder_config;        /*!< 编码器配置 */
    pid_ctrl_config_t pid_config;           /*!< PID配置 */
#ifdef USE_TB6612_DRIVER
    tb6612_config_t tb6612_config;          /*!< TB6612驱动配置 */
#endif
} mc_config_t;

#ifdef USE_TB6612_DRIVER
// TB6612模式默认配置
// 注意：TB6612只需要一个PWM信号，但espressif_bdc_motor库要求两个PWM GPIO都有效
// 因此我们提供一个虚拟的PWMB GPIO，但在实际控制中只使用PWMA
#define MC_CONFIG_DEFAULT(p, i, d) {                \
    .motor_config = {                               \
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,                       \
        .pwma_gpio_num = -1,  /* 实际PWM信号，将在main.c中设置 */ \
        .pwmb_gpio_num = -1,  /* 虚拟PWM信号，将在main.c中设置为有效GPIO */ \
    },                                              \
    .mcpwm_config = {                               \
        .group_id = 0,                              \
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,                  \
    },                                              \
    .tb6612_config = {                              \
        .pwm_gpio = -1,   /* PWM信号GPIO */         \
        .dir1_gpio = -1,  /* 方向控制1 GPIO */      \
        .dir2_gpio = -1,  /* 方向控制2 GPIO */      \
    },                                              \
    .pid_config = {                                 \
        .init_param = {                             \
            .kp = p,                                \
            .ki = i,                                \
            .kd = d,                                \
            .cal_type = PID_CAL_TYPE_INCREMENTAL,   \
            .max_output = PWM_MOTOR_DUTY_TICK_MAX - 2,    \
            .min_output = -(PWM_MOTOR_DUTY_TICK_MAX - 2), \
            .max_integral = 1000,                   \
            .min_integral = -1000,                  \
        }                                           \
    },                                              \
}
#else
// DRV8833模式默认配置
#define MC_CONFIG_DEFAULT(p, i, d) {                \
    .motor_config = {                               \
        .pwm_freq_hz = 25000,                       \
    },                                              \
    .mcpwm_config = {                               \
        .group_id = 0,                              \
        .resolution_hz = 10000000,                  \
    },                                              \
    .pid_config = {                                 \
        .init_param = {                             \
            .kp = p,                                \
            .ki = i,                                \
            .kd = d,                                \
            .cal_type = PID_CAL_TYPE_INCREMENTAL,   \
            .max_output = 10000000. / 25000 - 2,    \
            .min_output = -(10000000. / 25000 - 2), \
            .max_integral = 1000,                   \
            .min_integral = -1000,                  \
        }                                           \
    },                                              \
}
#endif

struct mc_t;
typedef struct mc_t *mc_handle_t;

esp_err_t mc_timer_loop_create(TickType_t pid_period_tick, TickType_t encoder_period_tick);
esp_err_t mc_timer_loop_del(void);
esp_err_t mc_timer_loop_start(void);
esp_err_t mc_timer_loop_stop(void);
esp_err_t mc_new(const mc_config_t *config, float max_rpm, mc_handle_t *handle);
esp_err_t mc_del(mc_handle_t handle);
esp_err_t mc_add_to_timer_loop(mc_handle_t handle);
esp_err_t mc_remove_from_timer_loop(mc_handle_t handle);
void mc_set_pid_params(mc_handle_t handle, float kp, float ki, float kd);
void mc_get_pid_params(mc_handle_t handle, float *kp, float *ki, float *kd);
void mc_set_expect_rpm(mc_handle_t handle, float expect_rpm);
float mc_get_real_rpm(mc_handle_t handle);
float mc_get_expect_rpm(mc_handle_t handle);

#ifdef __cplusplus
}
#endif // __cplusplus
