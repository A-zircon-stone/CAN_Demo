#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "string.h"

/* 串口打印任务：每20ms执行一次（50Hz） */
extern "C" void Printf_Task(void * argument)
{
    portTickType CurrentTime;
    /* Infinite loop */
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick

        char Robomaster[]="RM2027";
        //char uint8_t unsighed char?好吧不太懂这些类型。。。
         HAL_UART_Transmit(&huart1,(uint8_t*)Robomaster,strlen(Robomaster),1000);
        vTaskDelayUntil(&CurrentTime, 20/ portTICK_RATE_MS);  //每20ms执行一次（50Hz）
    }
}
