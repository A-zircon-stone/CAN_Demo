#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "string.h"
#include "Debug.hpp"
#include "canio.h"
extern volatile int16_t g_spdout;
/* 串口打印任务：每20ms执行一次（50Hz） */
extern "C" void Printf_Task(void * argument)
{
    portTickType CurrentTime;
    /* Infinite loop */
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick

        char send_buf[100];

        int length = snprintf(send_buf, sizeof(send_buf),
        "pid:%d,%d,%d,%d,%d\r\n",
                     200,
                     (int)motor2.Speed,
                     (int)g_spdout,
                     (int)motor2.Current,
                     (int)motor2.Angle
                              );

        HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, length, 100);
        //char uint8_t unsighed char?好吧不太懂这些类型。。。
       // usart_printf("RM2027\r\n");
        vTaskDelayUntil(&CurrentTime, 20/ portTICK_RATE_MS);  //每20ms执行一次（50Hz）
    }
}
