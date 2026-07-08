#ifndef BSP_TICK_H
#define BSP_TICK_H

#include "ti_msp_dl_config.h"

uint64_t systick_get_tick(void);
void systick_set_tick(uint64_t tick);
void systick_clear_tick(void);

extern int filt_velocity_r; // 滤波后的速度
extern int last_filt_velocitya_r;
extern int filt_velocity_l; // 滤波后的速度
extern int last_filt_velocitya_l;

#endif // BSP_TICK_H