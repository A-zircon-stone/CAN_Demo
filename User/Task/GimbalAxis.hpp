//
// Created by 24707 on 2026/8/29.
//

#ifndef CAN_DEMO_GIMBALAXIS_HPP
#define CAN_DEMO_GIMBALAXIS_HPP
#include "PID.hpp"
#include "canio.h"
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
        pid_.PID_Init(&pid_.PosParam, 0.3f, 0.0f, 0.0f);
        pid_.PID_Init(&pid_.SpdParam, 20.0f, 0.0f, 0.0f);
        pid_.PosParam.PID_OutMax = 200;
        pid_.PosParam.PID_OutStep = 5;
        pid_.PosParam.PID_ErrAllMax = 20000;
        pid_.PosParam.PID_Precision = 5;

        pid_.SpdParam.PID_OutMax = 2000;
        pid_.SpdParam.PID_OutStep = 200;
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
int16_t Axiscontrol()
{
    if (feedback==nullptr) return 0;
    int16_t target=target_angle;
    if (target>angle_max) target=angle_max;
    if (target<angle_min) target=angle_min;
    int16_t actual=feedback->Angle;
    int16_t error=angleError(target,actual);
    target_speed=(int16_t )pid_.SingleLoop_PID(&pid_.PosParam,0.0f ,error);
    int16_t Spdout=(int16_t )pid_.SingleLoop_PID(&pid_.SpdParam,feedback->Speed,target_speed);
    return Spdout;

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
    PIDc pid_;

};

extern Gimbal_Axis yaw;
extern Gimbal_Axis pitch;



#endif //CAN_DEMO_GIMBALAXIS_HPP
