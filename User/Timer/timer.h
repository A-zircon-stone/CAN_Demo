#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx_hal.h"

extern volatile uint8_t flag_2ms;

void Timer_Init(void);

#endif // TIMER_H
