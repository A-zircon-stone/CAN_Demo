#include "FreeRTOS.h"
#include "task.h"
#include "canio.h"
#include "PID.hpp"
#include "GimbalAxis.hpp"
#include "Debug.hpp"

/* 电机2反馈（main.cpp 里定义，CAN中断回调更新） */
extern "C" GM6020_Feedback_t motor2;
volatile int16_t g_spdout = 0;


static PIDc pid;                                // 对象
Gimbal_Axis yaw;
 Gimbal_Axis pitch;

/* 电机控制任务：每2ms执行一次（500Hz） */
extern "C" void Gimbal_Task(void *argument)
{   yaw.init(&motor2);
    yaw.set_target_angle(4000);
    yaw.set_angle_max(6880);

    yaw.set_angle_min(1200);
    pitch.init(&motor1);
 yaw.set_angle_max(7880);

 yaw.set_angle_min(2880);
    uart1_rx_start();



//     pid.PID_Init(&pid.SpdParam, 20.0f, 0.002f, 0.0f);
//     pid.PID_Init(&pid.PosParam, 0.3f, 0.005f, 0.0f);
//
//     pid.PosParam.PID_OutMax = 200;       // 位置环最大输出速度，先限制到 ±200rpm
//     pid.PosParam.PID_OutStep = 5;        // 目标速度变化别太猛
//     pid.PosParam.PID_ErrAllMax = 20000;      // 先不用积分
//     pid.PosParam.PID_Precision = 5;     // 误差小于 10 个编码值就认为到位
//
//     pid.SpdParam.PID_OutMax = 2000;
//     pid.SpdParam.PID_OutStep = 200;
//     pid.SpdParam.PID_ErrAllMax = 10000;
//     pid.SpdParam.PID_Precision = 2.0f;  // 初始化速度环
    //pid.PID_Init(&pid.PosParam, 0.0f, 0.0f, 0.0f);  // 初始化位置环

    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick
       // int16_t angle_error = GM6020_AngleError(200, motor2.Angle);

       // float Spdtarget = pid.SingleLoop_PID(&pid.PosParam,0.0f,(float)angle_error);
        //float SpdActual = pid.SingleLoop_PID(&pid.PosParam, (float)motor2.Angle, 200.0f);
        //假设目标角度200.0f
        // 接收到角度，放入Input（Actual）
        //float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, SpdActual);
       // float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, Spdtarget);

        //g_spdout = (int16_t)Spdout;

        //CAN_Send(0x1FE, 1500, 1500, 1500, 1500); // 给电机2发Spdout
        int16_t out = yaw.Axiscontrol();
     int16_t out_ = pitch.Axiscontrol();
        CAN_Send(0x1FE, out_, out, 0, 0);

        vTaskDelayUntil(&CurrentTime, 2/ portTICK_RATE_MS); 
    }// 每2ms执行一次（500Hz）
    }
