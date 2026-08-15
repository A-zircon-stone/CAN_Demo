#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t flag_2ms;

void Timer_Init(void);

#ifdef __cplusplus
}
#endif

#endif // TIMER_H
