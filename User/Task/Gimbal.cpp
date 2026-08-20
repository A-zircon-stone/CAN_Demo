#include "FreeRTOS.h"
#include "task.h"
#include "canio.h"
#include "PID.hpp"

/* 电机2反馈（main.cpp 里定义，CAN中断回调更新） */
extern "C" GM6020_Feedback_t motor2;

static PIDc pid;                                // 对象

/* 电机控制任务：每2ms执行一次（500Hz） */
extern "C" void Gimbal_Task(void *argument)
{
    pid.PID_Init(&pid.SpdParam, 0.0f, 0.0f, 0.0f);  // 初始化速度环
    pid.PID_Init(&pid.PosParam, 0.0f, 0.0f, 0.0f);  // 初始化位置环

    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick

        float SpdActual = pid.SingleLoop_PID(&pid.PosParam, (float)motor2.Angle, 200.0f);
        //假设目标角度200.0f
        // 接收到角度，放入Input（Actual）
        float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, SpdActual);

        CAN_Send(0x1FE, 0, (int16_t)Spdout, 0, 0);  // 给电机2发Spdout

        vTaskDelayUntil(&CurrentTime, 2/ portTICK_RATE_MS); 
    }// 每2ms执行一次（500Hz）
    }
