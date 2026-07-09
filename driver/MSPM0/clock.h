#ifndef _CLOCK_H_
#define _CLOCK_H_

extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);

#endif /* #ifndef _CLOCK_H_ */