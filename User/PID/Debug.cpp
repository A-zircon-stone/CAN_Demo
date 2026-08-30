//
// Created by 24707 on 2026/8/27.
//
#include "Debug.hpp"
#include "GimbalAxis.hpp"
#include <cstdio>
#include <cstring>

#define TX_BUF_SIZE 512
#define RX_BUF_SIZE 64

static uint8_t send_buf[TX_BUF_SIZE];
static uint8_t uart1_rx_buf[RX_BUF_SIZE];
static volatile uint8_t uart1_tx_busy = 0;

void uart1_rx_start(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, RX_BUF_SIZE) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

static void uart1_parse_rx(const uint8_t *data, uint16_t len)
{
    char line[RX_BUF_SIZE];
    uint16_t copy_len = len;

    if (copy_len >= RX_BUF_SIZE)
    {
        copy_len = RX_BUF_SIZE - 1;
    }

    memcpy(line, data, copy_len);
    line[copy_len] = '\0';

    char *p = line;
    while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t')
    {
        ++p;
    }

    int value = 0;
    if (sscanf(p, "yaw:%d", &value) == 1 ||
        sscanf(p, "yaw=%d", &value) == 1 ||
        sscanf(p, "y:%d", &value) == 1 ||
        sscanf(p, "y=%d", &value) == 1)
    {
        yaw.set_target_angle((int16_t)value);
    }
    else if (sscanf(p, "pitch:%d", &value) == 1 ||
             sscanf(p, "pitch=%d", &value) == 1 ||
             sscanf(p, "p:%d", &value) == 1 ||
             sscanf(p, "p=%d", &value) == 1)
    {
        pitch.set_target_angle((int16_t)value);
    }
    else if (sscanf(p, "%d", &value) == 1)
    {
        yaw.set_target_angle((int16_t)value);
    }
}

void usart_printf(const char *format, ...)
{
    va_list args;
    int length;

    if (uart1_tx_busy)
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf((char *)send_buf, TX_BUF_SIZE, format, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }

    if (length >= TX_BUF_SIZE)
    {
        length = TX_BUF_SIZE - 1;
    }

    if (HAL_UART_Transmit_DMA(&huart1, send_buf, (uint16_t)length) == HAL_OK)
    {
        uart1_tx_busy = 1;
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
        uart1_rx_start();
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size > 0)
        {
            uart1_parse_rx(uart1_rx_buf, Size);
        }

        uart1_rx_start();
    }
}
