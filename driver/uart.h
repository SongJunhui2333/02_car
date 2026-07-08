#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

#define UART_RX_MAX_LENGTH 100

// 函数声明
void UART_print_char(UART_Regs *uart, const uint8_t chr);
void UART_print_string(UART_Regs *uart, const char *str);
void UART_send_data(UART_Regs *uart, const uint8_t *buff, uint16_t length);

extern uint8_t debug_rx_flag;
extern __IO uint8_t uart_rx_buff[UART_RX_MAX_LENGTH];
#endif /* UART_H */
