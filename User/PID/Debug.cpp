//
// Created by 24707 on 2026/8/27.
//
#include "Debug.hpp"

#define TX_BUF_SIZE 512
static uint8_t send_buf[TX_BUF_SIZE];
void usart_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *)send_buf, TX_BUF_SIZE, (const char *)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)send_buf,length);
}