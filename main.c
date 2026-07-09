/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MSPM0/interrupt.h"
#include "OLED_Hardware_I2C/oled_hardware_i2c.h"
#include "Trace/Trace.h"
#include "Ultrasonic_Capture/ultrasonic_capture.h"
#include "WIT/wit.h"
#include "bsp_tick.h"
#include "key.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include <stdio.h>

extern uint8_t debug_rx_flag;
extern __IO uint8_t uart_rx_buff[UART_RX_MAX_LENGTH];
extern uint8_t uart_rx_length;
extern uint8_t uart_send_buff[100];
extern uint8_t print_rx_flag;

extern int filt_velocity_r;
extern int filt_velocity_l;

extern uint16_t motor_l_target_speed;
extern uint16_t motor_r_target_speed;

uint8_t oled_buffer[200];

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();

    // 电机初始化
    motor_init(1);
    motor_init(2);
    NVIC_EnableIRQ(DC_MOTOR_GPIOA_INT_IRQN); // 使能编码器中断
                                             // 电机pid初始化
    pid_init(&pid_motor_l, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);
    pid_init(&pid_motor_r, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);
    // 设置电机初始目标速度
    pid_set_setpoint(&pid_motor_l, motor_l_target_speed);
    pid_set_setpoint(&pid_motor_r, motor_r_target_speed);

    // 循迹pid初始化
    pid_init(&trace_pid, PID_POSITION, TRACE_KP, TRACE_KI, TRACE_KD, TRACE_MAX_OUT, TRACE_MIN_OUT);
    pid_set_setpoint(&trace_pid, TRACE_CENTER_POS); // 目标位置 = 传感器中心 4.5

    // 航向pid初始化（惯性导航用）
    pid_init(&pid_heading, PID_POSITION, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_MAX_OUT, HEADING_MIN_OUT);

    // 使能控制PID定时器中断
    // NVIC_EnableIRQ(CONTROL_PID_INST_INT_IRQN);

    // 使能UART接收中断
    NVIC_EnableIRQ(DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);

    Ultrasonic_Init(); // 初始化超声波测距模块

    WIT_Init(); // 初始化WIT传感器模块

    uint16_t distVal = 0;

    // motor_set_duty(1, 2000);
    // motor_set_duty(2, 2000);

    while (1)
    {
        // delay_ms(1000);
        motor_set_direction(1, 2);
        motor_set_direction(2, 2);
        // delay_ms(1000);
        // motor_set_direction(1, 2);

        // if (debug_rx_flag == 1)
        // {
        //     debug_rx_flag = 0;

        //     UART_print_string(DEBUG_INST, (char *)uart_rx_buff);
        //     UART_print_string(DEBUG_INST, "\r\n");

        //     uart_rx_length = 0;
        // }

        // /* 持续处理串口接收的数据 */

        // if (print_rx_flag)
        // {
        //     print_rx_flag = 0;

        //     UART_print_string(PRINT_INST, (char *)uart_send_buff);

        //     UART_print_string(PRINT_INST, "\r\n");

        //     uart_rx_length = 0;
        // }

        // UART_print_string(DEBUG_INST, "hello ti\n");
        // delay_ms(1000);

        distVal = Read_Ultrasonic();
        // sprintf((char *)oled_buffer, "%4u", distVal);
        sprintf((char *)oled_buffer, "%d\n,%d", filt_velocity_l, filt_velocity_r);
        OLED_ShowString(0, 0, oled_buffer, 16);
    }
}
