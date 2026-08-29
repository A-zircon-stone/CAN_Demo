#include "FreeRTOS.h"
#include "task.h"
#include "canio.h"
#include "PID.hpp"

/* 电机2反馈（main.cpp 里定义，CAN中断回调更新） */
extern "C" GM6020_Feedback_t motor2;
volatile int16_t g_spdout = 0;

static PIDc pid;                                // 对象
static int16_t GM6020_AngleError(int16_t target, int16_t actual)
{
    int16_t error = target - actual;

    if (error > 4096)
    {
        error -= 8192;
    }
    else if (error < -4096)
    {
        error += 8192;
    }

    return error;
}

/* 电机控制任务：每2ms执行一次（500Hz） */
extern "C" void Gimbal_Task(void *argument)
{
    pid.PID_Init(&pid.SpdParam, 20.0f, 0.002f, 0.0f);
    pid.PID_Init(&pid.PosParam, 0.3f, 0.005f, 0.0f);

    pid.PosParam.PID_OutMax = 200;       // 位置环最大输出速度，先限制到 ±200rpm
    pid.PosParam.PID_OutStep = 5;        // 目标速度变化别太猛
    pid.PosParam.PID_ErrAllMax = 20000;      // 先不用积分
    pid.PosParam.PID_Precision = 5;     // 误差小于 10 个编码值就认为到位

    pid.SpdParam.PID_OutMax = 2000;
    pid.SpdParam.PID_OutStep = 200;
    pid.SpdParam.PID_ErrAllMax = 10000;
    pid.SpdParam.PID_Precision = 2.0f;  // 初始化速度环
    //pid.PID_Init(&pid.PosParam, 0.0f, 0.0f, 0.0f);  // 初始化位置环

    portTickType CurrentTime;
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();  //读取当前时间下tick
        int16_t angle_error = GM6020_AngleError(200, motor2.Angle);

        float SpdActual = pid.SingleLoop_PID(&pid.PosParam,
                                             0.0f,
                                             (float)angle_error);
        //float SpdActual = pid.SingleLoop_PID(&pid.PosParam, (float)motor2.Angle, 200.0f);
        //假设目标角度200.0f
        // 接收到角度，放入Input（Actual）
        //float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, SpdActual);
        float Spdout    = pid.SingleLoop_PID(&pid.SpdParam, (float)motor2.Speed, SpdActual);

        g_spdout = (int16_t)Spdout;

        CAN_Send(0x1FE, 0, g_spdout, 0, 0); // 给电机2发Spdout

        vTaskDelayUntil(&CurrentTime, 2/ portTICK_RATE_MS); 
    }// 每2ms执行一次（500Hz）
    }
