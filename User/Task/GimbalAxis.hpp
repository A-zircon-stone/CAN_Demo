//
// Created by 24707 on 2026/8/29.
//

#ifndef CAN_DEMO_GIMBALAXIS_HPP
#define CAN_DEMO_GIMBALAXIS_HPP
#include "PID.hpp"
#include "canio.h"

extern volatile uint8_t g_protected_mode;

class Gimbal_Axis
{
    public:
    void init(GM6020_Feedback_t *fb)
    {
        feedback = fb;
        target_angle = fb->Angle;
        target_speed = 0;

        angle_min = 0;
        angle_max = 8191;
        pid_.PID_Init(&pid_.PosParam, 0.3f, 0.005f, 0.0f);
        pid_.PID_Init(&pid_.SpdParam, 20.0f, 0.002f, 0.0f);
        pid_.PosParam.PID_OutMax = 150;
        pid_.PosParam.PID_OutStep = 5;
        pid_.PosParam.PID_ErrAllMax = 20000;
        pid_.PosParam.PID_Precision = 5;

        pid_.SpdParam.PID_OutMax = 2000;
        pid_.SpdParam.PID_OutStep = 1500;
        pid_.SpdParam.PID_ErrAllMax = 10000;
        pid_.SpdParam.PID_Precision = 2;
    }

     int16_t get_angle()
     {  if (feedback==nullptr) return 0;
         return feedback->Angle;

     };

    void set_target_angle(int16_t angle)
    {target_angle=angle;};
    void set_angle_max(int16_t anglemax)
    {angle_max =anglemax;};
    void set_angle_min(int16_t anglemin)
    {angle_min=anglemin;};
    int16_t get_actual_speed()
    {
        actual_speed = feedback->Speed;
        return actual_speed;
    }


    void set_speed_mode(bool mode)
    {
        if (speed_mode == mode) return;

        speed_mode = mode;
        pid_.PID_Clear();
        target_speed = 0;

        if (!speed_mode)
        {
            speed_dir = 1;
        }
    }

    void set_speed_cmd(int16_t speedcmd)
    {
        if (speedcmd  >= 0)
        {
            speed_cmd = speedcmd ;
            speed_dir = 1;
        }
        else
        {
            speed_cmd = -speedcmd ;
            speed_dir = -1;
        }
    }

    void emergency_stop()
    {
        pid_.PID_Clear();
        target_speed = 0;
        speed_cmd = 0;
        speed_dir = 1;
        speed_mode = false;

        if (feedback != nullptr)
        {
            target_angle = feedback->Angle;
        }
    }
int16_t Axiscontrol()
{
    if (feedback==nullptr) return 0;

    if (g_protected_mode)
    {
        return 0;
    }

    if (!speed_mode)
    {
        int16_t target=target_angle;
        if (target>angle_max) target=angle_max;
        if (target<angle_min) target=angle_min;
        int16_t actual=feedback->Angle;
        int16_t error=angleError(target,actual);
        target_speed=(int16_t )pid_.SingleLoop_PID(&pid_.PosParam,0.0f ,error);
        int16_t Spdout=(int16_t )pid_.SingleLoop_PID(&pid_.SpdParam,feedback->Speed,target_speed);
        return Spdout;
    }
    else

    {
        int16_t actual=feedback->Angle;
        if (actual>=angle_max-limit_margin&&speed_dir==1)
        {
            speed_dir=-1;
            //speed_cmd =speed_dir *speed_cmd;
            pid_ .PID_Clear();
        }
        else if (actual<angle_min+limit_margin&&speed_dir==-1)
        {
            speed_dir =1;
            pid_ .PID_Clear();

        }
        target_speed =speed_cmd *speed_dir;

return (int16_t )pid_ .SingleLoop_PID(&pid_.SpdParam ,(float )feedback ->Speed,(float )target_speed );
    }


}

    int16_t get_target_angle() const { return target_angle; }


private:
    static int16_t angleError(int16_t target, int16_t actual)
    {
        int16_t error = target - actual;
        if (error > 4096) error -= 8192;
        else if (error < -4096) error += 8192;
        return error;
    }
    private:
    GM6020_Feedback_t *feedback;
    int16_t target_angle;
    int16_t target_speed;
    int16_t angle_max;
    int16_t angle_min;
    int16_t actual_speed;
    PIDc pid_;

    bool speed_mode = false;     // 当前是不是速度模式
    int16_t speed_cmd = 0;
    int8_t speed_dir = 1;        // 方向，1 或 -1
    int16_t limit_margin = 30;   // 限位提前量，防抖

};

extern Gimbal_Axis yaw;
extern Gimbal_Axis pitch;



#endif //CAN_DEMO_GIMBALAXIS_HPP
