#pragma once
#include <stdint.h>
#include "stm32f4xx.h"
struct DMA_HandleTypeDef {};
struct UART_HandleTypeDef { void *Instance; DMA_HandleTypeDef *hdmarx; };
extern UART_HandleTypeDef huart1;
#define USART1 reinterpret_cast<void *>(1)
#define HAL_OK 0
#define DMA_IT_HT 0
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)0)
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
