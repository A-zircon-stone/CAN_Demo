//
// Created by 24707 on 2026/8/27.
//
#include "Debug.hpp"
#include "GimbalAxis.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#define TX_BUF_SIZE 512
#define RX_BUF_SIZE 64

static uint8_t send_buf[TX_BUF_SIZE];
static uint8_t uart1_rx_buf[RX_BUF_SIZE];
static volatile uint8_t uart1_tx_busy = 0;
volatile uint8_t g_protected_mode = 1; // 上电保持零指令，Normal Mode 后才接受运动命令。

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

    char *end = p + strlen(p);
    while (end > p && (end[-1] == ' ' || end[-1] == '\r' || end[-1] == '\n' || end[-1] == '\t'))
    {
        --end;
    }
    *end = '\0';

    if (strcmp(p, "Protected Mode") == 0 || strcmp(p, "protected mode") == 0)
    {
        g_protected_mode = 1;
        motor2_current_setpoint = 0;
        motor2_position_mode = 0;
        ++motor2_command_revision;
    }
    else if (strcmp(p, "Normal Mode") == 0 || strcmp(p, "normal mode") == 0)
    {
        motor2_current_setpoint = 0;
        motor2_position_mode = 0;
        ++motor2_command_revision;
        g_protected_mode = 0;
    }
    else if (strcmp(p, "nn=learn") == 0 || strcmp(p, "nn=freeze") == 0 ||
             strcmp(p, "nn=off") == 0 || strcmp(p, "nn=reset") == 0)
    {
        // 这些命令只控制观察器，不启动运动，也不改变 PID 或目标角度。
        if (strcmp(p, "nn=learn") == 0)
            motor2_neural_mode = NeuralMode::Learn;
        else if (strcmp(p, "nn=freeze") == 0)
            motor2_neural_mode = NeuralMode::Frozen;
        else
            motor2_neural_mode = NeuralMode::Off;
        if (strcmp(p, "nn=reset") == 0)
            ++motor2_neural_reset_revision;
        ++motor2_neural_revision;
    }
    else if (g_protected_mode)
    {
        return;
    }
    else if (strncmp(p, "angle=", 6) == 0)
    {
        char *number_end;
        errno = 0;
        const long value = strtol(p + 6, &number_end, 10);
        if (number_end == p + 6 || *number_end != '\0' || errno == ERANGE ||
            value < 0 || value > 8191)
        {
            return;
        }
        motor2_target_angle = static_cast<int16_t>(value);
        motor2_current_setpoint = 0;
        motor2_position_mode = 1;
        ++motor2_command_revision;
    }
    else if (strncmp(p, "current=", 8) == 0)
    {
        char *number_end;
        errno = 0;
        const long value = strtol(p + 8, &number_end, 10);
        if (number_end == p + 8 || *number_end != '\0' || errno == ERANGE ||
            value < -16384 || value > 16384)
        {
            return;
        }
        motor2_current_setpoint = (int16_t)value;
        motor2_position_mode = 0;
        ++motor2_command_revision;
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
