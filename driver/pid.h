#ifndef PID_H
#define PID_H

#include "ti_msp_dl_config.h"

#define MOTOR_KP (0.5f)
#define MOTOR_KI (0.0f)
#define MOTOR_KD (0.0f)

/* PID控制模式 */
typedef enum
{
    PID_POSITION = 0,   /* 位置式PID */
    PID_INCREMENTAL = 1 /* 增量式PID */
} pid_type_t;

/* PID结构体 */
typedef struct
{
    /* PID参数 */
    float kp; /* 比例系数 */
    float ki; /* 积分系数 */
    float kd; /* 微分系数 */

    /* PID模式 */
    pid_type_t mode; /* PID控制模式: 位置式 / 增量式 */

    /* 目标值与反馈值 */
    float setpoint; /* 目标值（期望速度） */
    float feedback; /* 反馈值（实际速度） */

    /* 误差项 */
    float error;      /* 当前误差 */
    float last_error; /* 上一次误差 */
    float prev_error; /* 上上次误差（用于增量式） */

    /* 积分项 */
    float integral;     /* 积分累加值 */
    float integral_max; /* 积分限幅（抗积分饱和） */
    float integral_min;

    /* 输出限幅 */
    float output_max; /* 输出最大值 */
    float output_min; /* 输出最小值 */

    /* 输出值 */
    float output; /* PID计算输出 */
} pid_t;

/**
 * @brief 初始化PID控制器
 * @param pid     PID结构体指针
 * @param mode    PID模式（PID_POSITION 或 PID_INCREMENTAL）
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param max_out 输出最大值
 * @param min_out 输出最小值
 */
void pid_init(pid_t *pid, pid_type_t mode, float kp, float ki, float kd, float max_out, float min_out);

/**
 * @brief 设置PID目标值
 * @param pid       PID结构体指针
 * @param setpoint  目标值
 */
void pid_set_setpoint(pid_t *pid, float setpoint);

/**
 * @brief 设置PID参数
 * @param pid PID结构体指针
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 */
void pid_set_param(pid_t *pid, float kp, float ki, float kd);

/**
 * @brief PID计算核心函数
 * @param pid      PID结构体指针
 * @param feedback 当前反馈值（实际速度）
 * @return float   PID计算输出值
 */
float pid_calculate(pid_t *pid, float feedback);

/**
 * @brief 重置PID控制器（清零积分、误差等）
 * @param pid PID结构体指针
 */
void pid_reset(pid_t *pid);

/**
 * @brief 位置式PID计算
 * @param pid PID结构体指针
 * @return float 输出值
 */
float pid_position_calc(pid_t *pid);

/**
 * @brief 增量式PID计算
 * @param pid PID结构体指针
 * @return float 输出值
 */
float pid_incremental_calc(pid_t *pid);

#endif // PID_H