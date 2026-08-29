//
// Created by 24707 on 2026/8/29.
//

#ifndef CAN_DEMO_GIMBALAXIS_HPP
#define CAN_DEMO_GIMBALAXIS_HPP
#include "PID.hpp"
#include "canio.h"
class Gimbal_Axis
{
int get_angle(GM6020_Feedback_t *fb)
    {return fb->Angle;};
    void set_target_angle(int16_t angle,Gimbal_Axis *Yaw_Pitch)
    {Yaw_Pitch->target_angle=angle;};
    void set_angle_max(int16_t angle_max,Gimbal_Axis *Yaw_Pitch)
    {Yaw_Pitch->max_angle=angle_max;};
    void set_angle_min(int16_t angle_min,Gimbal_Axis *Yaw_Pitch)
    {Yaw_Pitch->min_angle=angle_min;};



    private:
    GM6020_Feedback_t *feedback;
    int16_t target_angle;
    int16_t target_speed;
    int16_t angle_max;
    int16_t angle_min;

};



#endif //CAN_DEMO_GIMBALAXIS_HPP
