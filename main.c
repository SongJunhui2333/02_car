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

#include "Ultrasonic_Capture/ultrasonic_capture.h"
#include "WIT/wit.h"
#include "bsp_tick.h"
#include "delay.h"
#include "irq.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
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
    OLED_ColorTurn(0);   // 0正常显示，1 反色显示
    OLED_DisplayTurn(0); // 0正常显示 1 屏幕翻转显示
    OLED_Clear();
    // NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    DL_ADC12_enableConversions(xuanniu_INST);
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);

    motor_init(1);
    motor_init(2);

    NVIC_EnableIRQ(DC_MOTOR_GPIOA_INT_IRQN); // 使能编码器中断

    NVIC_EnableIRQ(DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);

    // motor_set_duty(1, 3000);

    // 电机pid初始化
    pid_init(&pid_motor_l, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);
    pid_init(&pid_motor_r, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);

    pid_set_setpoint(&pid_motor_l, 20);
    pid_set_setpoint(&pid_motor_r, 20);

    Ultrasonic_Init();

    WIT_Init();

    uint16_t disVal = 0;

    // delay_ms(1000);

    while (1)
    {
        // delay_ms(1000);
        // motor_set_direction(1, 1);
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
        // delay_ms(50);

        // disVal = Read_Ultrasonic();
        // char disStr[20];
        // sprintf(disStr, "disVal: %4u", disVal);
        // OLED_ShowString(0, 0, (uint8_t *)disStr, 16);
        // OLED_Refresh();

        // OLED_ShowString(0, 0, (uint8_t *)"L:", 16);
        // char lStr[20];
        // sprintf(lStr, "%d", i);
        // i++;
        // OLED_ShowString(16, 0, (uint8_t *)lStr, 16);
        // OLED_Refresh();
        // delay_ms(100);

        sprintf((char *)oled_buffer, "pith:%-6.1f \n roll:%-6.1f \n yaw:%-6.1f\n\n", wit_data.pitch, wit_data.roll,
                wit_data.yaw);
        // OLED_ShowString(0, 0, oled_buffer, 16);
        // OLED_Refresh();

        UART_print_string(DEBUG_INST, oled_buffer);

        // sprintf((char *)oled_buffer, "%-6.1f", wit_data.roll);
        // // OLED_ShowString(0, 16, oled_buffer, 16);
        // // OLED_Refresh();
        // UART_print_string(DEBUG_INST, oled_buffer);
        // sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
        // // OLED_ShowString(0, 32, oled_buffer, 16);
        // // OLED_Refresh();
        // UART_print_string(DEBUG_INST, oled_buffer);
        delay_ms(1000);
    }
}
