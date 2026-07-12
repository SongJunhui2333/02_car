#include "clock.h"
#include "ti_msp_dl_config.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms)
        ;
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}
/**
 * @brief  延时函数，延时指定毫秒数
 * @param ms  延时的毫秒数
 */
void my_delay_ms(uint32_t ms)
{
    uint32_t cycles = (CPUCLK_FREQ / 1000) * ms;
    delay_cycles(cycles);
}
