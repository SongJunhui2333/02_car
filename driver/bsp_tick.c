#include "bsp_tick.h"
#include "delay.h"

static volatile uint64_t mTick = 0; // 当前滴答计数值

extern volatile unsigned long tick_ms;

__IO uint32_t speed_tick = 0; // 速度计数值

int filt_velocity_r = 0; // 滤波后的速度
int last_filt_velocitya_r = 0;

int filt_velocity_l = 0; // 滤波后的速度
int last_filt_velocitya_l = 0;

extern uint16_t encoder_l_count; // 编码器左轮计数
extern uint16_t encoder_r_count; // 编码器右轮计数

/**
 * @brief 增加系统滴答计数
 *
 * 这个函数由 SysTick 中断处理程序调用，用于增加系统滴答计数。它用于时间管理和延迟功能
 */
static void systick_inc_tick(void)
{
    mTick++;
    tick_ms++;
}

/**
 * @brief 获取当前系统计时器计数
 *
 * @return 当前系统时钟计数作为64位无符号整数
 */
uint64_t systick_get_tick(void)
{
    return mTick;
}

/**
 * @brief 设置系统计时器计数
 *
 * @param tick 要设置的系统时钟计数值
 */
void systick_set_tick(uint64_t tick)
{
    mTick = tick;
}

/**
 * @brief 清除系统计时器计数
 *
 * 这个函数将系统滴答计数重置为零。它可以用于重新初始化计时器或在特定情况下重置时间跟踪。
 */
void systick_clear_tick(void)
{
    mTick = 0;
}

// 滴答定时器中断服务函数
void SysTick_Handler(void)
{
    systick_inc_tick();
    Tick_SysTickCallback();
}