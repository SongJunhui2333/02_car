#include "Trace/Trace.h"

pid_t trace_pid; /* 轨迹PID结构体 */

uint8_t trace_sensor_state[Trace_Count] = {0}; /* 轨迹传感器状态数组 */

uint8_t trace_sensor_distance[Trace_Count] = {1, 2, 3, 4,
                                              5, 6, 7, 8}; /* 轨迹传感器距离数组，用于存储每个传感器的距离值 */
float trace_get_num = 0;                                   /* 检测到轨迹的传感器数量 */

volatile float trace_distance = 0;
volatile float last_trace_distance = 4.5;
float trace_base_speed = 20.0f; /* 循迹基础速度，可运行时修改 */

/**
 * @brief  计算角度偏移量
 * @param  Normal: 输入的8个unsigned short类型数据数组 field：1白底黑场 0黑底白场
 * @retval 计算得到的归一化返回值 返回值理论范围-7*1024~7*1024
 * @note   计算步骤：
 *         1. 对每个数据乘以其对应的权值(-7,-5,-3,-1,1,3,5,7)
 *         2. 求加权和
 *         3. 求原始数据和
 *         4. 用加权和除以原始数据和得到返回值
 */
int32_t CalculateNormalizedValue(unsigned short Normal[8], uint8_t field)
{
    // 定义权值数组，从左到右对应-7,-6,-4,-1,1,4,6,7
    const short weights[8] = {-7, -6, -4, -1, 1, 4, 6, 7};

    int32_t weighted_sum = 0;  // 加权和
    int32_t original_sum = 0;  // 原始数据和
    static int last_value = 0; // 上一次的值

    // 计算加权和和原始数据和
    for (int i = 0; i < 8; i++)
    {
        weighted_sum +=
            1024 * (field ? (4096 - Normal[i]) * weights[i] : Normal[i] * weights[i]); // 每个数据乘以对应权值
        original_sum += field ? (4096 - Normal[i]) : Normal[i];                        // 累加原始数据
    }

    // 计算并返回归一化值
    if (original_sum != 0)
    { // 避免除以0的情况
        last_value = weighted_sum / original_sum;
        return last_value;
    }
    else
    {
        return last_value; // 如果原始数据和为0，返回上一次的值
    }
}