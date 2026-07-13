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

#include "LED.h"
#include "MSPM0/interrupt.h"
#include "No_Mcu_Ganv_Grayscale_Sensor/No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "OLED_Hardware_I2C/oled_hardware_i2c.h"
#include "Trace/Trace.h"
#include "Ultrasonic_Capture/ultrasonic_capture.h"
#include "WIT/wit.h"
#include "bsp_tick.h"
#include "buzzer.h"
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
volatile uint8_t g_line_detected = 0; /* 黑线检测标志（中断中读取） */
volatile uint8_t g_stop_flag = 0;     /* 停止标志（中断中读取） */
uint8_t key_state_flag = 0;
uint8_t key_start_flag = 0;

extern uint8_t key_state_flag;
extern uint8_t key_start_flag;

/**
 * @brief 任务一：循迹+陀螺仪导航综合任务
 */
void task_1(void)
{
    static uint64_t ind_tick = 0;
    static uint64_t start_tick = 0; /* 任务启动时刻（用于0.5s防护） */
    static uint8_t prev_line = 0;
    static uint8_t has_ever_seen_line = 0;
    static uint8_t leave_count = 0;
    static uint8_t enc_count = 0;

    /* 蜂鸣器+LED亮1秒后自动熄灭（放在最前，停车后仍生效） */
    if (ind_tick > 0 && systick_get_tick() - ind_tick >= 1000)
    {
        led_off();
        buzzer_off();
    }

    /* 任务已完成则直接返回，防止重复执行 */
    if (g_stop_flag)
        return;

    if (ind_tick == 0)
    {
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        start_tick = ind_tick;
    }

    /* 任务开始前0.5秒：强制航向为0，跳过边沿检测 */
    if (systick_get_tick() - start_tick < 500)
    {
        heading_target = 0.0f;
        return;
    }

    if (g_line_detected)
        has_ever_seen_line = 1;

    if (!prev_line && g_line_detected)
    {
        enc_count++;
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
    }

    if (has_ever_seen_line && prev_line && !g_line_detected)
    {
        leave_count++;
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        if (leave_count >= 2)
        {
            g_stop_flag = 1;
            heading_target = 0.0f;
            start_tick = 0;
            prev_line = 0;
            has_ever_seen_line = 0;
            leave_count = 0;
            enc_count = 0;
        }
        else
        {
            heading_target = 180.0f;
        }
    }

    prev_line = g_line_detected;
}

void task_2(void)
{
    static uint64_t ind_tick = 0;
    static uint64_t start_tick = 0;
    static uint8_t prev_line = 0;
    static uint8_t has_ever_seen_line = 0;
    static uint8_t leave_count = 0;
    static uint8_t enc_count = 0;
    static uint8_t db_on = 0;    /* 有黑线消抖计数 */
    static uint8_t db_state = 0; /* 消抖后的黑线状态 */
    static float save_trace_spd; /* 保存其他任务的循迹速度 */
    static float save_gyro_spd;  /* 保存其他任务的陀螺仪速度 */
    static float enc2_heading;   /* 第2次遇黑线时的航向，用于后续离开 */

    if (ind_tick > 0 && systick_get_tick() - ind_tick >= 1000)
    {
        led_off();
        buzzer_off();
    }

    if (g_stop_flag)
        return;

    if (ind_tick == 0)
    {
        /* 保存并设置任务二的专属速度 */
        save_trace_spd = trace_base_speed;
        save_gyro_spd = gyro_base_speed;
        trace_base_speed = 20.0f; /* 任务二循迹速度 */
        gyro_base_speed = 30.0f;  /* 任务二陀螺仪速度 */

        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        start_tick = ind_tick;
    }

    if (systick_get_tick() - start_tick < 500)
    {
        heading_target = -38.0f;
        return;
    }

    /* ======== 第一层：原始信号 → 立即改变 heading_target ======== */
    uint8_t raw_line = g_line_detected;

    /* 上升沿：立即改变航向（不受消抖影响） */
    if (!prev_line && raw_line)
    {
        enc_count++;
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        if (enc_count == 1)
            heading_target = 90.0f;
        else if (enc_count == 2)
        {
            heading_target = 60.0f;
            enc2_heading = 90.0f; /* 保存第2次遇黑线航向 */
        }
    }

    /* 下降沿：仅在消抖确认过黑线后才判定离开 */
    if (has_ever_seen_line && prev_line && !raw_line)
    {
        leave_count++;
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        if (leave_count >= 2 && wit_data.yaw > 0.0f)
        {
            g_stop_flag = 1;
            heading_target = -38.0f;
            /* 恢复原始速度 */
            trace_base_speed = save_trace_spd;
            gyro_base_speed = save_gyro_spd;
            start_tick = 0;
            prev_line = 0;
            has_ever_seen_line = 0;
            leave_count = 0;
            enc_count = 0;
            db_on = 0;
            db_state = 0;
        }
        else
        {
            if (leave_count >= 2)
                heading_target = enc2_heading;
        }
    }

    prev_line = raw_line;

    /* ======== 第二层：消抖进入，立即退出 ======== */
    if (raw_line)
    {
        db_on++;
    }
    else
    {
        db_on = 0;
    }

    if (db_on >= 5)
    {
        db_state = 1;
        has_ever_seen_line = 1; /* 确认进入 */
        if (leave_count == 0)
        {
            heading_target = -130.0f; /* 第一次确认进入 → 预设离开航向 */
        }
    }
    if (!raw_line && has_ever_seen_line)
    {
        db_state = 0; /* 确认过进入 + 无黑线 → 立即退出 */
    }

    g_line_detected = db_state;
}

void task_3(void)
{
    static uint64_t ind_tick = 0;
    static uint64_t start_tick = 0;
    static uint8_t prev_line = 0;
    static uint8_t has_ever_seen_line = 0;
    static uint8_t leave_count = 0;
    static uint8_t enc_count = 0;
    static uint8_t db_on = 0;              /* 有黑线消抖计数 */
    static uint8_t db_state = 0;           /* 消抖后的黑线状态 */
    static float save_trace_spd;           /* 保存其他任务的循迹速度 */
    static float save_gyro_spd;            /* 保存其他任务的陀螺仪速度 */
    static float enc2_heading;             /* 第2次遇黑线时的航向，用于后续离开 */
    static uint8_t phase = 0;              /* 新增：0=初始循迹阶段, 1=原任务三流程 */
    static uint64_t no_line_tick = 0;      /* 进入无黑线区域的时刻（用于上升沿1秒判定） */
    static uint64_t rise_lockout_tick = 0; /* 上升沿冷却锁定期（防止抖动重复触发） */

    if (ind_tick > 0 && systick_get_tick() - ind_tick >= 1000)
    {
        led_off();
        buzzer_off();
    }

    if (g_stop_flag)
        return;

    if (ind_tick == 0)
    {
        /* 保存并设置任务三的专属速度 */
        save_trace_spd = trace_base_speed;
        save_gyro_spd = gyro_base_speed;
        trace_base_speed = 20.0f; /* 任务三循迹速度 */
        gyro_base_speed = 20.0f;  /* 任务三陀螺仪速度 */

        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        start_tick = ind_tick;
    }

    uint8_t raw_line = g_line_detected;

    /* ====== 阶段0：初始循迹（新增） ====== */
    if (phase == 0)
    {
        /* 小车在黑线上自然循迹（中断中根据 g_line_detected 自动处理） */

        /* 检测离开黑线（下降沿：prev_line && !raw_line） */
        if (prev_line && !raw_line)
        {
            /* 离开黑线 → 进入阶段1（原任务三流程） */
            phase = 1;
            start_tick = systick_get_tick(); /* 重置时刻，用于阶段1的500ms保护 */
            no_line_tick = start_tick;       /* 记录进入无黑线区域的时刻 */
            pid_reset(&trace_pid);           /* 离开黑线后清除循迹PID积分值 */
            led_on();                        /* 第一次进入无黑线区域，1秒声光提示 */
            buzzer_on();
            ind_tick = systick_get_tick();
            prev_line = 0;
            has_ever_seen_line = 0;
            heading_target = 142.5f; /* 设置陀螺仪导航目标航向 */
        }
        else
        {
            prev_line = raw_line;
        }
        return; /* 阶段0不执行后续的阶段1逻辑 */
    }

    /* ====== 阶段1：原任务三流程（不变） ====== */
    if (systick_get_tick() - start_tick < 500)
    {
        heading_target = 142.5f;
        return;
    }

    /* ======== 第一层：原始信号 → 立即改变 heading_target ======== */
    /* 上升沿：仅在无黑线行驶超过1秒 且 冷却期已过 才判定有效 */
    if (!prev_line && raw_line && (systick_get_tick() - no_line_tick >= 1000) &&
        (systick_get_tick() - rise_lockout_tick >= 500))
    {
        rise_lockout_tick = systick_get_tick(); /* 进入500ms冷却期，防止抖动重复触发 */
        enc_count++;
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        if (enc_count == 1)
            heading_target = -145.0f;
        else if (enc_count == 2)
        {
            heading_target = -10.0f;
            enc2_heading = -40.0f; /* 保存第2次遇黑线航向 */
        }
    }

    /* 下降沿：仅在消抖确认过黑线后才判定离开 */
    if (has_ever_seen_line && prev_line && !raw_line)
    {
        leave_count++;
        if (leave_count == 1)
        {
            no_line_tick = systick_get_tick(); /* 记录进入无黑线区域的时刻 */
            pid_reset(&trace_pid);             /* 离开黑线后清除循迹PID积分值 */
        }
        led_on();
        buzzer_on();
        ind_tick = systick_get_tick();
        if (leave_count >= 2 && wit_data.yaw < -90.0f)
        {
            g_stop_flag = 1;
            heading_target = 142.0f;
            /* 恢复原始速度 */
            trace_base_speed = save_trace_spd;
            gyro_base_speed = save_gyro_spd;
            start_tick = 0;
            prev_line = 0;
            has_ever_seen_line = 0;
            leave_count = 0;
            enc_count = 0;
            db_on = 0;
            db_state = 0;
            phase = 0; /* 新增：重置阶段，方便下次重新运行 */
            no_line_tick = 0;
            rise_lockout_tick = 0;
        }
        else
        {
            if (leave_count >= 2)
                heading_target = enc2_heading;
        }
    }

    prev_line = raw_line;

    /* ======== 第二层：消抖进入，立即退出 ======== */
    if (raw_line)
    {
        db_on++;
    }
    else
    {
        db_on = 0;
    }

    if (db_on >= 5)
    {
        db_state = 1;
        has_ever_seen_line = 1; /* 确认进入 */
        if (leave_count == 0)
        {
            heading_target = 45.5f; /* 第一次确认进入 → 预设离开航向 */
        }
    }
    if (!raw_line && has_ever_seen_line)
    {
        db_state = 0; /* 确认过进入 + 无黑线 → 立即退出 */
    }

    g_line_detected = db_state;
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();

    // 电机初始化
    motor_init(1);
    motor_init(2);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN); // 使能编码器中断
    NVIC_EnableIRQ(KEY_INT_IRQN);      // 使能按键中断
    // 电机pid初始化
    pid_init(&pid_motor_l, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);
    pid_init(&pid_motor_r, PID_INCREMENTAL, MOTOR_KP, MOTOR_KI, MOTOR_KD, 4000, 0);
    // 设置电机初始目标速度
    // sprintf((char *)send_speed, "%d\n", filt_velocity_l * 10);
    pid_set_setpoint(&pid_motor_l, motor_l_target_speed);
    pid_set_setpoint(&pid_motor_r, motor_r_target_speed);

    // 循迹pid初始化
    pid_init(&trace_pid, PID_POSITION, TRACE_KP, TRACE_KI, TRACE_KD, TRACE_MAX_OUT, TRACE_MIN_OUT);
    pid_set_setpoint(&trace_pid, 0); // 目标偏移量 = 0（黑线居中）

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

    OLED_ShowString(0, 2, (uint8_t *)"  Yaw", 8);

    // 根据黑白校准值初始化传感器
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    // 开启ADC中断
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    Tick_delay(100);

    while (1)
    {

        // 读取超声波测距值并显示在OLED上
        distVal = Read_Ultrasonic();
        sprintf((char *)oled_buffer, "dist: %4u", distVal);
        OLED_ShowString(0, 2, oled_buffer, 16);

        // 显示按键状态和启动/停止标志位
        sprintf((char *)oled_buffer, "%d,%d", key_start_flag, key_state_flag);
        OLED_ShowString(5 * 8, 7, oled_buffer, 8);

        // 显示WIT传感器数据

        sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
        OLED_ShowString(5 * 8, 0, oled_buffer, 16);
        sprintf((char *)oled_buffer, "target:%-6.1f", heading_target);
        OLED_ShowString(0, 4, oled_buffer, 16);

        // 传感器数据处理
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Get_Normalize_For_User(&sensor, Normal);

        // 前0.5秒不更新黑线检测，消除上电时传感器干扰
        if (systick_get_tick() >= 500)
        {
            g_line_detected = (Digtal != 0xFF);
        }

        /* 按键控制任务选择与启停 */
        {
            static uint8_t prev_key = 0;
            if (key_start_flag && !prev_key)
            {
                g_stop_flag = 0; /* 按键按下，清除停止标志 */
            }
            prev_key = key_start_flag;
        }

        if (key_start_flag)
        {
            switch (key_state_flag)
            {
            case 1:
                task_1();
                break;
            case 2:
                task_2();
                break;
            case 3:
                task_3();
                break;
            default:
                break;
            }
        }
        else
        {
            g_stop_flag = 1;
        }

        motor_set_direction(1, 2);
        motor_set_direction(2, 2);

        // UART_print_string(PRINT_INST, "Hello TI!\n");
        // delay_ms(500);

        // // 蓝牙串口检查代码
        // if (debug_rx_flag)
        // {
        //     debug_rx_flag = 0;
        //     UART_print_string(PRINT_INST, (char *)uart_rx_buff);
        //     UART_print_string(PRINT_INST, "\n");
        //     uart_rx_length = 0;
        //     // memset(uart_rx_buff, 0, UART_RX_MAX_LENGTH);
        // }
        // if (print_rx_flag)
        // {
        //     print_rx_flag = 0;
        //     // UART_print_string(PRINT_INST, (char *)uart_rx_buff);
        //     // UART_print_string(PRINT_INST, "\n");
        //     UART_print_string(DEBUG_INST, (char *)uart_rx_buff);
        //     UART_print_string(DEBUG_INST, "\n");
        //     uart_rx_length = 0;
        //     // memset(uart_rx_buff, 0, UART_RX_MAX_LENGTH);
        // }

        // printf("Anolog%d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
        // sprintf((char *)rx_buff, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n", (Digtal >> 0) & 0x01, (Digtal >> 1) & 0x01,
        //         (Digtal >> 2) & 0x01, (Digtal >> 3) & 0x01, (Digtal >> 4) & 0x01, (Digtal >> 5) & 0x01,
        //         (Digtal >> 6) & 0x01, (Digtal >> 7) & 0x01);
        // UART_print_string(DEBUG_INST, (char *)rx_buff);
        // memset(rx_buff, 0, 256);
        // sprintf((char *)rx_buff, "result_angle:%d\n\n", result_angle);
        // UART_print_string(DEBUG_INST, rx_buff);
        // memset(rx_buff, 0, 256);
        // delay_ms(50);

        // led_on();
        // buzzer_on();
        // delay_ms(1000);
        // led_off();
        // buzzer_off();
        // delay_ms(1000);

        // pid_set_setpoint(&pid_motor_l, motor_l_target_speed);
        // pid_set_setpoint(&pid_motor_r, motor_r_target_speed);
    }
}
