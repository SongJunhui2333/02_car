#include "Trace/Trace.h"

pid_t trace_pid; /* 轨迹PID结构体 */

uint8_t trace_sensor_state[Trace_Count] = {0}; /* 轨迹传感器状态数组 */

uint8_t trace_sensor_distance[Trace_Count] = {1, 2, 3, 4,
                                              5, 6, 7, 8}; /* 轨迹传感器距离数组，用于存储每个传感器的距离值 */
float trace_get_num = 0;                                   /* 检测到轨迹的传感器数量 */

volatile float trace_distance = 0;
volatile float last_trace_distance = 4.5;

/**
 * @brief 判断当前是否检测到黑线
 * @return 0表示未检测到黑线，1表示检测到黑线
 */
uint8_t trace_black_line_detect(void)
{
    trace_reflash();

    for (int i = 0; i < Trace_Count; i++)
    {
        if (trace_sensor_state[i] != 0)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 读取轨迹传感器状态
 * @param gpio_port 轨迹传感器所在的GPIO端口
 * @param sensor_pin 轨迹传感器对应的GPIO引脚
 * @return 1表示未检测轨迹，0表示检测到轨迹
 */
uint8_t read_trace_sensor_state(GPIO_Regs *gpio_port, uint32_t sensor_pin)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, sensor_pin); // 0x00000040 0b01000000 PB6 0~31
    if ((high_bits & sensor_pin) != 0)
        return 1;
    else
        return 0;
}

void trace_reflash(void)
{
    trace_get_num = 0; /* 重置检测到轨迹的传感器数量 */

    if (!read_trace_sensor_state(TRACE_Sensor_1_PORT, TRACE_Sensor_1_PIN))
    {
        trace_sensor_state[0] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[0] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_2_PORT, TRACE_Sensor_2_PIN))
    {
        trace_sensor_state[1] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[1] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_3_PORT, TRACE_Sensor_3_PIN))
    {
        trace_sensor_state[2] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[2] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_4_PORT, TRACE_Sensor_4_PIN))
    {
        trace_sensor_state[3] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[3] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_5_PORT, TRACE_Sensor_5_PIN))
    {
        trace_sensor_state[4] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[4] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_6_PORT, TRACE_Sensor_6_PIN))
    {
        trace_sensor_state[5] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[5] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_7_PORT, TRACE_Sensor_7_PIN))
    {
        trace_sensor_state[6] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[6] = 0;
    }
    if (!read_trace_sensor_state(TRACE_Sensor_8_PORT, TRACE_Sensor_8_PIN))
    {
        trace_sensor_state[7] = 1;
        trace_get_num++;
    }
    else
    {
        trace_sensor_state[7] = 0;
    }

    if (trace_get_num == 0)
    {
        trace_get_num = 1; /* 防止除以零 */
    }

    trace_distance = 0; /* 重置轨迹距离 */
    for (int i = 0; i < Trace_Count; i++)
    {
        trace_distance += trace_sensor_state[i] * trace_sensor_distance[i]; /* 计算轨迹距离 */
    }
    trace_distance = trace_distance / trace_get_num; /* 计算平均轨迹距离 */

    if (trace_distance != 0)
    {
        last_trace_distance = trace_distance; /* 更新最后一次有效的轨迹距离 */
    }
    else if (trace_distance == 0)
    {
        trace_distance = last_trace_distance; /* 如果当前轨迹距离为0，则使用最后一次有效的轨迹距离 */
    }
}

