#include "interrupt.h"
#include "../No_Mcu_Ganv_Grayscale_Sensor/No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "../Trace/Trace.h"
#include "../WIT/wit.h"
#include "../bsp_tick.h"
#include "../motor.h"
#include "../uart.h"
#include "clock.h"
#include "key.h"
#include "stdio.h"

/* 外部声明：电机目标速度（在pid.c中定义） */
extern uint16_t motor_l_target_speed;
extern uint16_t motor_r_target_speed;

extern unsigned char rx_buff[256];

/* 外部声明：灰度传感器全局变量（在main.c中定义） */
extern No_MCU_Sensor sensor;
extern unsigned short Normal[8];
extern unsigned char Digtal;
extern int result_angle;

/* 外部声明：黑线检测标志（在main.c中更新） */
extern volatile uint8_t g_line_detected;
extern volatile uint8_t g_stop_flag;

uint16_t encoder_l_count = 0;
uint16_t encoder_r_count = 0;

/* 创建左右电机PID控制器实例 */
pid_t pid_motor_l;
pid_t pid_motor_r;

/* 创建航向PID控制器实例（用于惯性导航） */
pid_t pid_heading;

/* ========== 惯性导航状态变量 ========== */
uint8_t gyro_mode_active = 0;  /* 是否处于陀螺仪导航模式 */
float heading_target = 0.0f;   /* 目标航向角，由用户在main.c中手动设置 */
float gyro_base_speed = 20.0f; /* 陀螺仪导航基础速度，可运行时修改 */

extern uint8_t key_state_flag;
extern uint8_t key_start_flag;

void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case DC_MOTOR_ENCODER2_A_IIDX: // 编码器2（电机2=左轮）
        encoder_l_count++;
        break;
    case DC_MOTOR_ENCODER1_A_IIDX: // 编码器1（电机1=右轮）
        encoder_r_count++;
        break;
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case KEY_K_1_IIDX: {
        // 处理 Key_State_pin0 的中断
        my_delay_ms(10);                          // 简单的消抖处理
        if (get_key_state(KEY_PORT, KEY_K_1_PIN)) // 确认按键仍然被按下
        {
            key_state_flag = (key_state_flag + 1) % 5; // 设置按键状态标志位
        }
    }
    break;
    case KEY_K_2_IIDX: {
        // 处理 Key_State_pin1 的中断
        my_delay_ms(10);                          // 简单的消抖处理
        if (get_key_state(KEY_PORT, KEY_K_2_PIN)) // 确认按键仍然被按下
        {
            key_state_flag = (key_state_flag - 1 + 5) % 5; // 设置按键状态标志位
        }
    }
    break;
    case KEY_K_3_IIDX: {
        // 处理 Key_State_pin2 的中断
        my_delay_ms(10);                          // 简单的消抖处理
        if (get_key_state(KEY_PORT, KEY_K_3_PIN)) // 确认按键仍然被按下
        {
            key_start_flag = !key_start_flag; // 切换启动/停止标志位
        }
    }
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

        // filt_velocity_l =
        //     a * encoder_l_count +
        //     (1 - a) * last_filt_velocitya_l;     //
        //     简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        // last_filt_velocitya_l = filt_velocity_l; // 此次速度记录为“上次速度”

        // filt_velocity_r =
        //     a * encoder_r_count +
        //     (1 - a) * last_filt_velocitya_r;     //
        //     简单算法滤波，此次速度取30%的权重，过往速度取70%的权重，让速度更平滑
        // last_filt_velocitya_r = filt_velocity_r; // 此次速度记录为“上次速度”

        // encoder_l_count = 0;
        // encoder_r_count = 0;

        // // uint8_t send_speed[20];
        // // sprintf((char *)send_speed, "%d,%d\n", filt_velocity_l, filt_velocity_r);
        // // UART_print_string(DEBUG_INST, (char *)send_speed);

        // /* 使用编码器计数值作为速度反馈进行PID计算 */
        // float ctrl_l = pid_calculate(&pid_motor_l, (float)filt_velocity_l);
        // float ctrl_r = pid_calculate(&pid_motor_r, (float)filt_velocity_r);
        // if (ctrl_l > 20)
        //     ctrl_l = 20;
        // if (ctrl_r > 20)
        //     ctrl_r = 20;

        // /* 将PID输出转换为电机占空比并施加到电机 */
        // /* 注意：motor(1)=右轮, motor(2)=左轮，此处做映射交换 */
        // motor_set_duty(2, (uint16_t)(100 * ctrl_l)); // ctrl_l → 左轮(motor 2)
        // motor_set_duty(1, (uint16_t)(100 * ctrl_r)); // ctrl_r → 右轮(motor 1)

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
 *   模式1 - 循迹PID：检测到黑线时，使用result_angle计算转向修正
 *   模式2 - 陀螺仪惯性导航：丢失黑线时，锁定当前航向，航向PID保持直线
 *
 * @note 灰度传感器读取和黑线检测在main.c的while循环中完成，
 *       中断只读取全局变量result_angle和g_line_detected进行PID运算。
 */
void CONTROL_PID_INST_IRQHandler(void)
{
    float a = 0.3; // 滤波系数，取值范围为0~1，越接近1，滤波效果越明显
    switch (DL_Timer_getPendingInterrupt(CONTROL_PID_INST))
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

        // uint8_t send_speed[20];
        // sprintf((char *)send_speed, "%d,%d\n", filt_velocity_l, filt_velocity_r);
        // UART_print_string(DEBUG_INST, (char *)send_speed);

        if (g_stop_flag)
        {
            motor_set_duty(1, 0);
            motor_set_duty(2, 0);
            pid_set_setpoint(&pid_motor_l, 0);
            pid_set_setpoint(&pid_motor_r, 0);
            break;
        }

        float steering = 0;
        float base_speed;

        Digtal = Get_Digtal_For_User(&sensor);

        if (g_line_detected)
        {
            /* ====== 循迹PID模式 ====== */

            result_angle = CalculateNormalizedValue(Normal, 1) / 10;
            pid_set_setpoint(&trace_pid, 0);
            steering = pid_calculate(&trace_pid, (float)result_angle);
            float left_speed = trace_base_speed - steering;
            float right_speed = trace_base_speed + steering;
            if (left_speed < 0.0f)
                left_speed = 0.0f;
            if (right_speed < 0.0f)
                right_speed = 0.0f;
            if (left_speed > 45.0f)
                left_speed = 45.0f;
            if (right_speed > 45.0f)
                right_speed = 45.0f;
            pid_set_setpoint(&pid_motor_l, left_speed);
            pid_set_setpoint(&pid_motor_r, right_speed);
            // sprintf((char *)rx_buff, "result_angle:%d,%.2f,%.2f,%d,%d\n", result_angle, left_speed, right_speed,
            //         filt_velocity_l, filt_velocity_r);
            // UART_print_string(DEBUG_INST, rx_buff);
            // memset(rx_buff, 0, 256);
        }
        else
        {
            /* ====== 模式2：陀螺仪惯性导航 ====== */
            base_speed = gyro_base_speed;
            if (!gyro_mode_active)
            {
                gyro_mode_active = 1;
                pid_reset(&pid_heading);
            }
            float heading_error = heading_target - wit_data.yaw;
            if (heading_error > 180.0f)
                heading_error -= 360.0f;
            if (heading_error < -180.0f)
                heading_error += 360.0f;
            pid_set_setpoint(&pid_heading, 0);
            steering = pid_calculate(&pid_heading, heading_error);

            /* 差速控制 */
            float left_speed = base_speed + steering;
            float right_speed = base_speed - steering;
            if (left_speed < 0.0f)
                left_speed = 0.0f;
            if (right_speed < 0.0f)
                right_speed = 0.0f;
            if (left_speed > 45.0f)
                left_speed = 45.0f;
            if (right_speed > 45.0f)
                right_speed = 45.0f;
            pid_set_setpoint(&pid_motor_l, left_speed);
            pid_set_setpoint(&pid_motor_r, right_speed);

            // sprintf((char *)rx_buff,
            //         "heading_target:%.2f,\nbase_speed:%.2f,\nsteering:%.2f,\nheading_error:%.2f,\n%.2f,\n%.2f,\n%d,\n%"
            //         "d\n\n",
            //         heading_target, base_speed, steering, heading_error, left_speed, right_speed, filt_velocity_l,
            //         filt_velocity_r);
            // UART_print_string(DEBUG_INST, (char*)rx_buff);
            // memset(rx_buff, 0, 256);

            // /* ====== 无黑线 → 立即停车（循迹调试时启用） ====== */
            // motor_set_duty(1, 0);
            // motor_set_duty(2, 0);
            // pid_set_setpoint(&pid_motor_l, 0);
            // pid_set_setpoint(&pid_motor_r, 0);
        }
        /* 使用编码器计数值作为速度反馈进行PID计算 */
        float ctrl_l = pid_calculate(&pid_motor_l, (float)filt_velocity_l);
        float ctrl_r = pid_calculate(&pid_motor_r, (float)filt_velocity_r);
        if (ctrl_l > 20)
            ctrl_l = 20;
        if (ctrl_r > 20)
            ctrl_r = 20;

        /* 将PID输出转换为电机占空比并施加到电机 */
        /* 注意：motor(1)=右轮, motor(2)=左轮，此处做映射交换 */
        motor_set_duty(2, (uint16_t)(100 * ctrl_l)); // ctrl_l → 左轮(motor 2)
        motor_set_duty(1, (uint16_t)(100 * ctrl_r)); // ctrl_r → 右轮(motor 1)

        break;
    }

    default:
        break;
    }
}