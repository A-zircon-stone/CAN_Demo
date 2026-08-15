/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "canio.h"
#include "timer.h"
#include "PID.hpp"


/* 外部函数：SystemClock_Config 定义在 CubeMX 生成的 main.c 里 */
extern "C" void SystemClock_Config(void);

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();

  GM6020_Feedback_t motor2;
  Timer_Init();   // 2ms 定时中断
  CAN_Init();     // CAN 滤波器 + 启动
PIDc pid;                                // 对象
pid.PID_Init(&pid.SpdParam, 0.0f, 0.0f, 0.0f);  // 初始化速度环
pid.PID_Init(&pid.PosParam, 0.0f, 0.0f, 0.0f);  // 初始化位置环

  /* Infinite loop */
  while (1)
  {
    if (flag_2ms) {
      flag_2ms = 0;

      CAN_Receive(&motor2);              // 非阻塞: 有数据就更新motor2
float SpdActual = pid.SingleLoop_PID(&pid.PosParam, (float)motor2.Angle, 200.0f);  // 接收到角度，放入Input（Actual）
float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, SpdActual);

    CAN_Send(0x1FE, 0, (int16_t)Spdout, 0, 0);  // 给电机2发Spdout
    }
  }
}
