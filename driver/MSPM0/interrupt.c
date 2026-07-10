#include "interrupt.h"
#include "../Trace/Trace.h"
#include "../WIT/wit.h"
#include "../bsp_tick.h"
#include "../motor.h"
#include "../uart.h"
#include "stdio.h"

/* 外部声明：电机目标速度（在pid.c中定义） */
extern uint16_t motor_l_target_speed;
extern uint16_t motor_r_target_speed;

uint16_t encoder_l_count = 0;
uint16_t encoder_r_count = 0;

/* 创建左右电机PID控制器实例 */
pid_t pid_motor_l;
pid_t pid_motor_r;

/* 创建航向PID控制器实例（用于惯性导航） */
pid_t pid_heading;

/* ========== 惯性导航状态变量 ========== */
static float heading_target = 0.0f;  /* 目标航向角，丢失黑线时锁定 */
static uint8_t gyro_mode_active = 0; /* 是否处于陀螺仪导航模式 */

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
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
    float a = 0.3; // 滤波系数，取值范围为0~1，越接近1，滤波效果越明显

    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD: {

        filt_velocity_l =
            a * encoder_l_count +
            (1 - a) * last_filt_velocitya_l;     // 简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        last_filt_velocitya_l = filt_velocity_l; // 此次速度记录为“上次速度”

        filt_velocity_r =
            a * encoder_r_count +
            (1 - a) * last_filt_velocitya_r;     // 简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        last_filt_velocitya_r = filt_velocity_r; // 此次速度记录为“上次速度”

        encoder_l_count = 0;
        encoder_r_count = 0;

        uint8_t send_speed[20];
        sprintf((char *)send_speed, "%d,%d\n", filt_velocity_l, filt_velocity_r);
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

void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if (DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while (rxSize >= 11)
    {
        checkSum = 0;
        for (int i = packCnt * 11; i < (packCnt + 1) * 11 - 1; i++)
            checkSum += wit_dmaBuffer[i];

        if ((wit_dmaBuffer[packCnt * 11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt * 11 + 10]))
        {
            if (wit_dmaBuffer[packCnt * 11 + 1] == 0x51)
            {
                wit_data.ax =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) / 2.048; // mg
                wit_data.ay =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) / 2.048; // mg
                wit_data.az =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) / 2.048; // mg
                wit_data.temperature =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 9] << 8) | wit_dmaBuffer[packCnt * 11 + 8]) / 100.0; // °C
            }
            else if (wit_dmaBuffer[packCnt * 11 + 1] == 0x52)
            {
                wit_data.gx =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) / 16.384; // °/S
                wit_data.gy =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) / 16.384; // °/S
                wit_data.gz =
                    (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) / 16.384; // °/S
            }
            else if (wit_dmaBuffer[packCnt * 11 + 1] == 0x53)
            {
                wit_data.roll = (int16_t)((wit_dmaBuffer[packCnt * 11 + 3] << 8) | wit_dmaBuffer[packCnt * 11 + 2]) /
                                32768.0 * 180.0; // °
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt * 11 + 5] << 8) | wit_dmaBuffer[packCnt * 11 + 4]) /
                                 32768.0 * 180.0; // °
                wit_data.yaw = (int16_t)((wit_dmaBuffer[packCnt * 11 + 7] << 8) | wit_dmaBuffer[packCnt * 11 + 6]) /
                               32768.0 * 180.0; // °
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt * 11 + 9] << 8) | wit_dmaBuffer[packCnt * 11 + 8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }

    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t)&wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}

/**
 * @brief 控制PID中断服务函数（50ms触发一次）
 *
 * 两种模式自动切换：
 *   - 检测到黑线 → 循迹PID（位置式，trace_distance反馈）
 *   - 丢失黑线 → 陀螺仪航向保持PID（wit_data.yaw反馈）
 */
void CONTROL_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CONTROL_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD: {
        float steering;
        float base_speed;

        /* 刷新循迹传感器，判断是否有黑线 */
        trace_reflash();
        uint8_t line_detected = trace_black_line_detect();

        if (line_detected)
        {
            /* ====== 模式1：循迹模式 ====== */
            gyro_mode_active = 0; // 退出陀螺仪模式
            base_speed = (float)TRACE_BASE_SPEED;

            /* 死区处理：黑线在中心附近直行，避免抖动 */
            if (trace_distance >= TRACE_CENTER_POS - TRACE_DEAD_ZONE &&
                trace_distance <= TRACE_CENTER_POS + TRACE_DEAD_ZONE)
            {
                pid_set_setpoint(&pid_motor_l, base_speed);
                pid_set_setpoint(&pid_motor_r, base_speed);
                break;
            }

            /* 位置式PID：setpoint=4.5, feedback=trace_distance */
            steering = pid_calculate(&trace_pid, trace_distance);
        }
        else
        {
            /* ====== 模式2：陀螺仪惯性导航 ====== */
            base_speed = (float)GYRO_BASE_SPEED;

            if (!gyro_mode_active)
            {
                /* 首次丢失黑线 → 锁定当前航向为目标 */
                gyro_mode_active = 1;
                heading_target = wit_data.yaw;
                pid_reset(&pid_heading); // 清空陀螺仪PID历史
            }

            /* 计算最短航向误差，处理 ±180° 跳变
             *  例: target=170°, current=-170° → error=340° → 归一化= -20°
             *  表示实际只偏了20°，往正方向转更短 */
            float heading_error = heading_target - wit_data.yaw;
            if (heading_error > 180.0f)
                heading_error -= 360.0f;
            if (heading_error < -180.0f)
                heading_error += 360.0f;

            /* 航向PID：setpoint=0, feedback=heading_error
             *  → error = 0 - heading_error = -heading_error
             *  → 当heading_error>0（偏右），输出<0（左转），自动纠偏 */
            pid_set_setpoint(&pid_heading, 0);
            steering = pid_calculate(&pid_heading, heading_error);
        }

        /* ====== 差速控制：左右轮差速转向 ====== */
        float left_speed = base_speed + steering;
        float right_speed = base_speed - steering;

        /* 限幅保护 */
        if (left_speed < 0.0f)
            left_speed = 0.0f;
        if (right_speed < 0.0f)
            right_speed = 0.0f;
        if (left_speed > 20.0f)
            left_speed = 20.0f;
        if (right_speed > 20.0f)
            right_speed = 20.0f;

        /* 更新电机PID目标速度 */
        pid_set_setpoint(&pid_motor_l, left_speed);
        pid_set_setpoint(&pid_motor_r, right_speed);

        break;
    }

    default:
        break;
    }
}