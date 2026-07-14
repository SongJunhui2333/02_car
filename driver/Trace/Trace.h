#ifndef TRACE_H
#define TRACE_H

#include "pid.h"
#include "ti_msp_dl_config.h"

#define Trace_Count 8 // 循迹模块路数

/* ========== 循迹PID参数（可调） ========== */
#define TRACE_KP (0.008f * 0.7f) /* 比例系数 — 越大转向越灵敏 */
#define TRACE_KI (0.04f)         /* 积分系数 — 循迹一般不需要，先保持0 */
#define TRACE_KD (0.03f * 0.3f)  /* 微分系数 — 抑制震荡，让循迹更平滑 */
#define TRACE_MAX_OUT (70.0f)    /* 输出上限 */
#define TRACE_MIN_OUT (-70.0f)   /* 输出下限 */

/* 循迹传感器中心位置（8路传感器位置1~8，中心=4.5） */
// #define TRACE_CENTER_POS (4.5f)

/* 死区阈值：当偏离中心小于此值时，认为已居中，不转向 */
// #define TRACE_DEAD_ZONE (0.3f)

/* 基础行驶速度（循迹时的默认电机目标速度，变量可在运行时修改） */
// 旧宏定义已移除，改用变量 trace_base_speed

extern pid_t trace_pid;               /* 轨迹PID结构体 */
extern volatile float trace_distance; /* 当前黑线位置（供外部使用） */
extern float trace_base_speed;        /* 循迹基础速度（可运行时修改） */

uint8_t trace_black_line_detect(void);
int32_t CalculateNormalizedValue(unsigned short Normal[8], uint8_t field);

#endif // TRACE_H