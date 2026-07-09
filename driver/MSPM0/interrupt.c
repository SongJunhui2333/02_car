#include "interrupt.h"
#include "../WIT/wit.h"
#include "../bsp_tick.h"
#include "../motor.h"
#include "../uart.h"
#include "stdio.h"

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

        // uint8_t send_speed[20];
        // sprintf((char *)send_speed, "%d\n", filt_velocity_l * 10);
        // UART_print_string(DEBUG_INST, (char *)send_speed);

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
