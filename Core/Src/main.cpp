/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "canio.h"
#include "timer.h"
#include "cmsis_os.h"
#include "Debug.hpp"
#include "dma.h"

/* 外部函数：SystemClock_Config 定义在 CubeMX 生成的 main.c 里 */
extern "C" void SystemClock_Config(void);

/* FreeRTOS 初始化函数（CubeMX 生成在 freertos.c 里） */
extern "C" void MX_FREERTOS_Init(void);

/* 全局变量：电机2反馈（中断回调里收数据，main循环里读数据） */
extern "C" {
GM6020_Feedback_t motor2;
GM6020_Feedback_t motor1;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();

  CAN_Init();     // CAN 滤波器 + 启动

  
  osKernelInitialize();   // 初始化 FreeRTOS 内核
   MX_FREERTOS_Init();     // 创建任务
  osKernelStart();        // 启动调度器（之后不再返回）

  /* Infinite loop */
  while (1)
  {
    // usart_printf(
    //     "M2 Angle=%d,Speed=%d,Current=%d,Temp=%u\r\n",
    //     (int)motor1.Angle,
    //     (int)motor1.Speed,
    //     (int)motor1.Current,
    //     (unsigned int)motor1.Temperature
    // );


    //  HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
    // CAN_Send(0x1FE, 0, 1500, 0, 0);
    // HAL_Delay(30);



  }
}
