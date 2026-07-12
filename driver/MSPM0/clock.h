#ifndef _CLOCK_H_
#define _CLOCK_H_

#include "ti_msp_dl_config.h"

extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void my_delay_ms(uint32_t ms);

#endif /* #ifndef _CLOCK_H_ */