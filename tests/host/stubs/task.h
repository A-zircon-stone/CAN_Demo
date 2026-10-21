#pragma once
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t *, TickType_t);
