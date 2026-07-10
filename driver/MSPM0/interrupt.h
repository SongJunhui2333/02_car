#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "../pid.h"
#include "ti_msp_dl_config.h"

/* ========== 航向PID参数（惯性导航-可调） ========== */
#define HEADING_KP (0.2f)        /* 比例系数 — 越大纠偏越猛 */
#define HEADING_KI (0.0f)        /* 积分系数 — 消除稳态误差 */
#define HEADING_KD (1.0f)        /* 微分系数 — 抑制震荡 */
#define HEADING_MAX_OUT (50.0f)  /* 转向修正上限 */
#define HEADING_MIN_OUT (-50.0f) /* 转向修正下限 */

/* 惯性导航模式下的基础行驶速度 */
#define GYRO_BASE_SPEED (30)

/* 外部声明：左右电机PID控制器实例 */
extern pid_t pid_motor_l;
extern pid_t pid_motor_r;

/* 外部声明：航向PID控制器实例 */
extern pid_t pid_heading;

/* 外部声明：编码器计数值 */
extern uint16_t encoder_l_count;
extern uint16_t encoder_r_count;

#endif // INTERRUPT_H
