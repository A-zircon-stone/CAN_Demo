#ifndef EXAMPLE_PID_HPP
#define EXAMPLE_PID_HPP

#define PID_DEFAULT_PRECISION 0.0f         //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 3000000         //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 10192     //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3192 //输出微分限幅


typedef struct
{
    float Kp1;
    float Ki1;
    float Kd1;

    float PID_Err_now;
    float PID_Err_last;
    float PID_Err_all;

    float PID_Out;
    float PID_lastout;
    float PID_Target;      //PID的目标值
    float PID_lastTarget;
    float PID_Input;       //PID输入

    float PID_Precision; // PID最小精度 精度？
    float PID_ErrAllMax; // PID积分限幅
    float PID_OutMax;     // PID输出限幅
    float PID_OutStep;     // PID输出步幅限制
} PID_t;

class PIDc {
private:
    void PID_Update(PID_t *WhichPID,float NowInput,float Target);
    float PID_Calculate(PID_t *WhichPID);
public:
    void PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd);
    float Pos_Spd_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Input,float Pos_Target,float Spd_Input);
    float SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target);
    void PID_Clear();
public:
    PID_t SpdParam;
    PID_t PosParam;
};

#endif //EXAMPLE_PID_HPP
