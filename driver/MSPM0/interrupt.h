#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "../pid.h"
#include "ti_msp_dl_config.h"

/* 外部声明：左右电机PID控制器实例 */
extern pid_t pid_motor_l;
extern pid_t pid_motor_r;

/* 外部声明：编码器计数值 */
extern uint16_t encoder_l_count;
extern uint16_t encoder_r_count;

#endif // INTERRUPT_H
