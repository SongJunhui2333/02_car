#include "LED.h"

/**
 * @brief 打开LED
 */
void led_on(void)
{
    DL_GPIO_setPins(LIGHT_PORT, LIGHT_LED_PIN);
}

/**
 * @brief 关闭LED
 */
void led_off(void)
{
    DL_GPIO_clearPins(LIGHT_PORT, LIGHT_LED_PIN);
}

/**
 * @brief 切换LED状态
 */
void led_toggle(void)
{
    DL_GPIO_togglePins(LIGHT_PORT, LIGHT_LED_PIN);
}
