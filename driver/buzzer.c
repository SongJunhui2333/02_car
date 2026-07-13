#include "buzzer.h"

// 蜂鸣器高电平触发
/**
 * @brief 打开蜂鸣器
 */
void buzzer_on(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_buzzer_PIN);
}

/**
 * @brief 关闭蜂鸣器
 */
void buzzer_off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_buzzer_PIN);
}

/**
 * @brief 切换蜂鸣器状态
 */
void buzzer_toggle(void)
{
    DL_GPIO_togglePins(BUZZER_PORT, BUZZER_buzzer_PIN);
}
