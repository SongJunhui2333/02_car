#include "Trace/Trace.h"

pid_t trace_pid; /* 轨迹PID结构体 */

uint8_t trace_sensor_state[Trace_Count] = {0}; /* 轨迹传感器状态数组 */

uint8_t trace_sensor_distance[Trace_Count] = {1, 2, 3, 4,
                                              5, 6, 7, 8}; /* 轨迹传感器距离数组，用于存储每个传感器的距离值 */
float trace_get_num = 0;                                   /* 检测到轨迹的传感器数量 */

volatile float trace_distance = 0;
volatile float last_trace_distance = 4.5;


