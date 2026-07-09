#include "motor.h"

/**
 * @brief Initializes the motor driver and sets the initial state of the motor.
 *
 * This function configures the GPIO pins and timer for the specified motor.
 * It enables the motor driver and sets the initial PWM duty cycle to 0.
 *
 * @param motor_id The ID of the motor to initialize (1 or 2).
 */
void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN); // 使能电机驱动器

    DL_Timer_startCounter(PWM_MOTOR_INST); // 启动PWM计数器

    if (motor_id == 1)
    {
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, 0, GPIO_PWM_MOTOR_C0_IDX);
    }
    else if (motor_id == 2)
    {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, 0, GPIO_PWM_MOTOR_C1_IDX);
    }

    DL_Timer_startCounter(MOTOR_PID_INST);   // 启动PID定时器计数器
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN); // 使能PID定时器中断
}

void motor_set_duty(uint8_t motor_id, uint16_t duty)
{

    if (duty > 4000)
    {
        duty = 4000; // 限制占空比最大值为4000
    }
    if (duty < 0)
    {
        duty = 0; // 限制占空比最小值为0
    }

    if (motor_id == 1)
    {
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, duty, GPIO_PWM_MOTOR_C0_IDX);
    }
    else if (motor_id == 2)
    {
        DL_Timer_setCaptureCompareValue(PWM_MOTOR_INST, duty, GPIO_PWM_MOTOR_C1_IDX);
    }
}

/**
 * @brief Sets the direction of the specified motor.
 *
 * This function controls the direction of the motor by setting the appropriate GPIO pins.
 *
 * @param motor_id The ID of the motor to control (1 or 2).
 * @param direction The desired direction (0: stop, 1: forward, 2: reverse).
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1)
    {
        if (direction == 0)
        {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 2)
        {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if (motor_id == 2)
    {
        if (direction == 0)
        {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 1)
        {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 2)
        {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}