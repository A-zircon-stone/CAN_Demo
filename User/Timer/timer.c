#include "timer.h"

volatile uint8_t flag_2ms = 0;

void Timer_Init(void)
{
    // TIM6: 定时 2ms (TIM6_CLK = 84MHz, APB1 × 2)
    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->PSC = 8399;                   // 84MHz / 8400 = 10kHz
    TIM6->ARR = 19;                     // 20 tick × 0.1ms = 2ms
    TIM6->DIER |= TIM_DIER_UIE;         // 使能更新中断
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;           // 启动
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;         // 清中断标志
        flag_2ms = 1;
    }
}
