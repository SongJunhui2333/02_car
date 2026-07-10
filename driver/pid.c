#include "pid.h"

uint16_t motor_l_target_speed = 30;
uint16_t motor_r_target_speed = 30;

/**
 * @brief 初始化PID控制器
 */
void pid_init(pid_t *pid, pid_type_t mode, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->mode = mode;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;

    pid->integral = 0.0f;
    pid->integral_max = max_out * 0.8f; /* 积分限幅默认为输出的80% */
    pid->integral_min = min_out * 0.8f;

    pid->output_max = max_out;
    pid->output_min = min_out;

    pid->output = 0.0f;
}

/**
 * @brief 设置PID目标值
 */
void pid_set_setpoint(pid_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief 设置PID参数
 */
void pid_set_param(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief PID计算核心函数
 *        根据模式选择位置式或增量式计算
 */
float pid_calculate(pid_t *pid, float feedback)
{
    pid->feedback = feedback;

    if (pid->mode == PID_POSITION)
    {
        return pid_position_calc(pid);
    }
    else
    {
        return pid_incremental_calc(pid);
    }
}

/**
 * @brief 位置式PID计算
 *
 * 计算公式:
 *   error = setpoint - feedback
 *   integral += error * dt
 *   derivative = error - last_error
 *   output = Kp * error + Ki * integral + Kd * derivative
 *
 * 带积分抗饱和和输出限幅
 */
float pid_position_calc(pid_t *pid)
{
    /* 计算当前误差 */
    pid->error = pid->setpoint - pid->feedback;

    /* 积分累加（带抗饱和限幅） */
    pid->integral += pid->error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;

    /* 微分项 */
    float derivative = pid->error - pid->last_error;

    /* PID输出计算 */
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    /* 积分抗饱和（条件积分法）：
     * 当输出饱和时，停止积分增长，防止积分饱和 */
    if (pid->output >= pid->output_max || pid->output <= pid->output_min)
    {
        /* 如果输出饱和，将积分回退一步 */
        pid->integral -= pid->error;
    }

    /* 更新误差历史 */
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 增量式PID计算
 *
 * 计算公式:
 *   error = setpoint - feedback
 *   delta_output = Kp*(error - last_error) + Ki*error + Kd*(error - 2*last_error + prev_error)
 *   output += delta_output
 *
 * 增量式PID输出的是控制量的增量，不会引起积分饱和问题，
 * 且对执行机构冲击小，适合电机速度控制
 */
float pid_incremental_calc(pid_t *pid)
{
    /* 计算当前误差 */
    pid->error = pid->setpoint - pid->feedback;

    /* 计算增量输出 */
    float delta_output = pid->kp * (pid->error - pid->last_error) + pid->ki * pid->error +
                         pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);

    /* 累加增量到输出 */
    pid->output += delta_output;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    /* 更新误差历史 */
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 重置PID控制器
 */
void pid_reset(pid_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->feedback = 0.0f;
}
