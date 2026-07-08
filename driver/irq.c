#include "irq.h"
#include "bsp_tick.h"
#include "motor.h"
#include "stdio.h"
#include "uart.h"

int status = 0;

uint16_t encoder_l_count = 0;
uint16_t encoder_r_count = 0;

/* 创建左右电机PID控制器实例 */
pid_t pid_motor_l;
pid_t pid_motor_r;

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY9_IIDX:
        status = (status + 1) % 3;
        break;
    case KEY_KEY10_IIDX:
        status = (status + 3 - 1) % 3;
        break;
    case DC_MOTOR_ENCODER2_A_IIDX: // 编码器2A引脚中断
        encoder_r_count++;
        break;

    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_ENCODER1_A_IIDX: // 编码器1A引脚中断
        encoder_l_count++;
        break;

    default:
        break;
    }
}

/**
 * @brief PID定时器中断处理函数
 *
 * 该函数在PID定时器中断发生时被调用，用于执行PID控制计算。
 * 它根据编码器计数值计算电机的速度，并使用PID算法计算控制输出。
 * 然后将计算得到的控制输出应用到电机的占空比上，从而实现对电机速度的调节。
 */
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD: {

        uint8_t send_speed[20];
        sprintf((char *)send_speed, "%d\n", filt_velocity_l * 10);
        UART_print_string(DEBUG_INST, (char *)send_speed);

        /* 使用编码器计数值作为速度反馈进行PID计算 */
        float ctrl_l = pid_calculate(&pid_motor_l, (float)filt_velocity_l);
        float ctrl_r = pid_calculate(&pid_motor_r, (float)filt_velocity_r);

        /* 将PID输出转换为电机占空比并施加到电机 */
        motor_set_duty(1, (uint16_t)(100 * ctrl_l));
        motor_set_duty(2, (uint16_t)(100 * ctrl_r));

        break;
    }

    default:
        break;
    }
}