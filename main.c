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
#include "No_Mcu_Ganv_Grayscale_Sensor/No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "OLED_Hardware_I2C/oled_hardware_i2c.h"
#include "Trace/Trace.h"
#include "Ultrasonic_Capture/ultrasonic_capture.h"
#include "WIT/wit.h"
#include "bsp_tick.h"
#include "delay.h"
#include "key.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include <stdio.h>

unsigned short Anolog[8] = {0};
unsigned short white[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
unsigned short black[8] = {300, 300, 300, 300, 300, 300, 300, 300};
unsigned short Normal[8];
unsigned char rx_buff[256] = {0};

extern uint8_t debug_rx_flag;
extern __IO uint8_t uart_rx_buff[UART_RX_MAX_LENGTH];
extern uint8_t uart_rx_length;
extern uint8_t uart_send_buff[100];
extern uint8_t print_rx_flag;
// 初始化
No_MCU_Sensor sensor;
unsigned char Digtal;

extern int filt_velocity_r;
extern int filt_velocity_l;

extern uint16_t motor_l_target_speed;
extern uint16_t motor_r_target_speed;

uint8_t oled_buffer[200];

int result_angle = 0;

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
    // sprintf((char *)send_speed, "%d\n", filt_velocity_l * 10);
    pid_set_setpoint(&pid_motor_l, motor_l_target_speed);
    pid_set_setpoint(&pid_motor_r, motor_r_target_speed);

    // 循迹pid初始化
    pid_init(&trace_pid, PID_POSITION, TRACE_KP, TRACE_KI, TRACE_KD, TRACE_MAX_OUT, TRACE_MIN_OUT);
    pid_set_setpoint(&trace_pid, TRACE_CENTER_POS); // 目标位置 = 传感器中心 4.5

    // 航向pid初始化（惯性导航用）
    pid_init(&pid_heading, PID_POSITION, HEADING_KP, HEADING_KI, HEADING_KD, HEADING_MAX_OUT, HEADING_MIN_OUT);

    // 使能控制PID定时器中断
    NVIC_EnableIRQ(CONTROL_PID_INST_INT_IRQN);

    // 使能UART接收中断
    NVIC_EnableIRQ(DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);

    Ultrasonic_Init(); // 初始化超声波测距模块

    WIT_Init(); // 初始化WIT传感器模块

    uint16_t distVal = 0;

    OLED_ShowString(0, 7, (uint8_t *)"WIT Demo", 8);

    OLED_ShowString(0, 0, (uint8_t *)"Pitch", 8);
    OLED_ShowString(0, 2, (uint8_t *)" Roll", 8);
    OLED_ShowString(0, 4, (uint8_t *)"  Yaw", 8);

    // 根据黑白校准值初始化传感器
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    // 开启ADC中断
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    Tick_delay(100);

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

        // 陀螺仪测试代码
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.pitch);
        OLED_ShowString(5 * 8, 0, oled_buffer, 16);
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.roll);
        OLED_ShowString(5 * 8, 2, oled_buffer, 16);
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
        OLED_ShowString(5 * 8, 4, oled_buffer, 16);
        delay_ms(50);

        // 传感器数据处理
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Get_Normalize_For_User(&sensor, Normal);
        Digtal = Get_Digtal_For_User(&sensor);
        // 计算角度线性偏移量
        result_angle = CalculateNormalizedValue(Normal, 1);
        // printf("Anolog
        // %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
        sprintf((char *)rx_buff, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n", (Digtal >> 0) & 0x01, (Digtal >> 1) & 0x01,
                (Digtal >> 2) & 0x01, (Digtal >> 3) & 0x01, (Digtal >> 4) & 0x01, (Digtal >> 5) & 0x01,
                (Digtal >> 6) & 0x01, (Digtal >> 7) & 0x01);
        UART_print_string(DEBUG_INST, (char *)rx_buff);
        memset(rx_buff, 0, 256);
        sprintf((char *)rx_buff, "result_angle:%d\n\n", result_angle);
        UART_print_string(DEBUG_INST, rx_buff);
        memset(rx_buff, 0, 256);
        delay_ms(1000);
    }
}
