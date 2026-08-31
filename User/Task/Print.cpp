#include "board.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "string.h"
#include "Debug.hpp"
#include "canio.h"
#include "GimbalAxis.hpp"
extern Gimbal_Axis yaw;
extern Gimbal_Axis pitch;
extern volatile int16_t g_spdout;
/* 串口打印任务：每20ms执行一次（50Hz） */
extern "C" void Printf_Task(void * argument)
{
    portTickType CurrentTime;
    /* Infinite loop */
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick

        //usart_printf("pid:%d,%d,%d,%d,%d\r\n",
                     // 200,
                     // (int)motor2.Speed,
                     // (int)g_spdout,
                     // (int)motor2.Current,
                     // (int)motor2.Angle);
        //char uint8_t unsighed char?好吧不太懂这些类型。。。
      //usart_printf("RM2027\r\n");
        usart_printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
              yaw.get_target_angle(),
              yaw.get_angle(),
              pitch.get_target_angle(),
              pitch.get_angle(),
              yaw.get_actual_speed(),
              pitch.get_actual_speed(),
              yaw.Axiscontrol(),
              pitch.Axiscontrol()


              );

        // usart_printf("%d\r\n",
        //
        //      motor1.Angle);


        // static uint16_t cnt = 0;
        //
        // if (BOARD_ROLE == BOARD_ROLE_MASTER)
        // {
        //     cnt++;
        //     if (cnt >= 50)   // 50次 * 20ms = 1s
        //     {
        //         cnt = 0;
        //         CAN_Send(BoardHello , 0, 0, 0, 0);
        //         int16_t angle_cdeg = 1234;   // 12.34°
        //         //CAN_Send(Board_Angle1, angle_cdeg, 0, 0, 0);
        //     }
        //
        // }
        // usart_printf("link:%u\r\n", board_link_ok);//判断链路是否正常
        vTaskDelayUntil(&CurrentTime, 20/ portTICK_RATE_MS);  //每20ms执行一次（50Hz）
    }
}
