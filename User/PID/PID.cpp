
#include "PID.hpp"

/*
* @name   PID_SpeedParamInit
* @brief  初始化PID速度环的参数
* @param  WhichPID PID结构体指针
* @retval None
*/
void PIDc::PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd)
{ //初始化PID的默认参数
    WhichPID->Kp1 = Kp;
    WhichPID->Ki1 = Ki;
    WhichPID->Kd1 = Kd;

    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/*
* @name   PID_Update
* @brief  更新PID的数据，即更新PID的输入值
* @param  WhichPID PID结构体指针,NowInput 当前PID输入值
* @retval None
*/
void PIDc::PID_Update(PID_t *WhichPID,float NowInput,float Target)
{//NowInput:Actual
    WhichPID->PID_Input = NowInput;
    WhichPID->PID_Target = Target;
    WhichPID->PID_Err_last = WhichPID->PID_Err_now;
    WhichPID->PID_Err_now = WhichPID->PID_Target - WhichPID->PID_Input;

    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision){
        WhichPID->PID_Err_now = 0;
    }  //数值在PID最小精度之内      落在[-a,a]

    WhichPID->PID_Err_all += WhichPID->PID_Err_now;
    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax){   //积分限幅
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    }
    else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax){
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }
}

/*
* @name   PID_Calculate
* @brief  位置式PID，注意不是指位置环PID
* @param  WhichPID PID结构体指针
* @retval None
*/
float PIDc::PID_Calculate(PID_t *WhichPID){
    WhichPID->PID_Out =
            WhichPID->Kp1 * WhichPID->PID_Err_now +
            WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last);
    WhichPID->PID_Out += (WhichPID->PID_Err_all * WhichPID->Ki1);

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)   //PID输出限幅
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)    //PID输出步长限制
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;

    WhichPID->PID_lastout = WhichPID->PID_Out;
    return WhichPID->PID_Out;
}

/*
* @name   Pos_Spd_PID
* @brief  串级PID
* @param  WhichPID PID结构体指针
* @retval None
*/
float PIDc::Pos_Spd_PID(PID_t *Spdparam,PID_t *Posparam, float Pos_Target, float Pos_Input, const float Spd_Input){
    PID_Update(Posparam,Pos_Input,Pos_Target);
    PID_Calculate(Posparam);//先进行位置环PID运算（外环）
    PID_Update(Spdparam,Spd_Input,Posparam->PID_Out);  //Target如此赋值是为让位置环的Error_Now和input相等，Error_Now=Target - NowInput
    PID_Calculate(Spdparam);//后进行速度环PID运算（内环）
    return Spdparam->PID_Out;
}

/*
 * 单环PID
 */
float PIDc::SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target){
    PID_Update(Which_PID,Pos_Input,Pos_Target);
    PID_Calculate(Which_PID);

    return Which_PID->PID_Out;
}

/*
 * @brief 清零所有PID数据
 */
void PIDc::PID_Clear(){
    SpdParam.PID_Err_now = 0.0f;
    SpdParam.PID_Err_all = 0.0f;
    SpdParam.PID_Err_last = 0.0f;
    SpdParam.PID_Out = 0.0f;

    PosParam.PID_Err_now = 0.0f;
    PosParam.PID_Err_all = 0.0f;
    PosParam.PID_Err_last = 0.0f;
    PosParam.PID_Out = 0.0f;
}