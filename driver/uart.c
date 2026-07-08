#include "uart.h"

void UART_print_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

/**
 * @brief   打印字符串
 *
 * @param uart   UART寄存器结构体指针
 * @param str   要打印的字符串，必须以'\0'结尾
 */
void UART_print_string(UART_Regs *uart, const char *str)
{
    while (*str)
    {
        UART_print_char(uart, (uint8_t)*str);
        str++;
    }
}

/**
 * @brief   发送数据
 * @param uart  UART寄存器结构体指针
 * @param buff      要发送的数据缓冲区
 * @param length    要发送的数据长度
 */
void UART_send_data(UART_Regs *uart, const uint8_t *buff, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        DL_UART_transmitDataBlocking(uart, buff[i]);
    }
}

uint8_t uart_send_buff[100];
uint8_t uart_send_data[100];


__IO uint8_t uart_rx_buff[UART_RX_MAX_LENGTH];
uint8_t uart_rx_length = 0;

uint8_t debug_rx_flag = 0;

uint8_t rx = 0;
int16_t a = 0;
float b = 0;
uint32_t test = 0;
__IO uint32_t capture_value = 0;

__IO uint16_t timea_counter = 0;

__IO uint16_t rx_size = 0;

void DEBUG_INST_IRQHandler()
{
    volatile uint32_t res = DL_UART_Main_getPendingInterrupt(DEBUG_INST);
    switch (res)
    {
    case DL_UART_MAIN_IIDX_RX: {
        while (DL_UART_receiveDataCheck(DEBUG_INST, (uint8_t *)(uart_rx_buff + uart_rx_length)))
        {
            uart_rx_length++;
            if (uart_rx_length >= UART_RX_MAX_LENGTH)
            {
                break;
            }
        }
    }
    case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR: {
        while (DL_UART_receiveDataCheck(DEBUG_INST, (uint8_t *)(uart_rx_buff + uart_rx_length)))
        {
            uart_rx_length++;
            if (uart_rx_length >= UART_RX_MAX_LENGTH)
            {
                break;
            }
        }
        debug_rx_flag = 1;
    }
    break;
    }
}
