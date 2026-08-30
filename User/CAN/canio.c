//
// Created by 24707 on 2026/8/8.
//

#include "canio.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "board.hpp"
extern CAN_HandleTypeDef hcan1;
volatile int16_t board_angle2 = 0;

void CAN_Init()
{
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;    //启用过滤器
    CAN_Filter.FilterBank = 0;//CAN1 有 14 个滤波器（编号 0~13）
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN_Filter.FilterIdHigh = Motor_1 << 5;
    CAN_Filter.FilterIdLow = Motor_1 << 5;
    CAN_Filter.FilterMaskIdHigh = 0x000 << 5;
    CAN_Filter.FilterMaskIdLow = 0x000 << 5;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.SlaveStartFilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void CAN_Send(int16_t ID, int16_t Mess_1, int16_t Mess_2, int16_t Mess_3, int16_t Mess_4)
{
    uint32_t Send_Mailbox=0;//硬件发送邮箱？不太懂
    CAN_TxHeaderTypeDef TxHeader;//发送头结构体
    TxHeader.IDE = CAN_ID_STD;//标准帧
    TxHeader.RTR = CAN_RTR_DATA;//数据帧  控制帧是请求？
    TxHeader.DLC = 8;
    TxHeader.StdId = ID;
    uint8_t CAN_Tx_Mess[8];//8Bit数组满足gm6020电机协议要求
    CAN_Tx_Mess[0]=Mess_1>>8;//位运算，右移八位取mess的高八位存入第一个数
    CAN_Tx_Mess[1]=Mess_1;//直接存入取低八位
    CAN_Tx_Mess[2]=Mess_2>>8;
    CAN_Tx_Mess[3]=Mess_2;
    CAN_Tx_Mess[4]=Mess_3>>8;
    CAN_Tx_Mess[5]=Mess_3;
    CAN_Tx_Mess[6]=Mess_4>>8;
    CAN_Tx_Mess[7]=Mess_4;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, CAN_Tx_Mess, &Send_Mailbox);//调用hal发送函数
}

void CAN_Receive(GM6020_Feedback_t *feedback,GM6020_Feedback_t *feedback2)
{

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t CAN_Rx_Mess[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CAN_Rx_Mess);

    uint16_t ID = RxHeader.StdId;

    switch (ID)
    {
    case Motor_2:
        feedback->Angle       = (int16_t)((CAN_Rx_Mess[0] << 8) | CAN_Rx_Mess[1]);
        feedback->Speed       = (int16_t)((CAN_Rx_Mess[2] << 8) | CAN_Rx_Mess[3]);
        feedback->Current     = (int16_t)((CAN_Rx_Mess[4] << 8) | CAN_Rx_Mess[5]);
        feedback->Temperature = CAN_Rx_Mess[6];
        break;
    case Motor_1:
        feedback2->Angle       = (int16_t)((CAN_Rx_Mess[0] << 8) | CAN_Rx_Mess[1]);
        feedback2->Speed       = (int16_t)((CAN_Rx_Mess[2] << 8) | CAN_Rx_Mess[3]);
        feedback2->Current     = (int16_t)((CAN_Rx_Mess[4] << 8) | CAN_Rx_Mess[5]);
        feedback2->Temperature = CAN_Rx_Mess[6];
        break;
    case BoardHello:
        if (BOARD_ROLE == BOARD_ROLE_SLAVE)
        {
            CAN_Send(BoardRespect , 0, 0, 0, 0);
        }
        break;

    case BoardRespect:
        board_link_ok = 1;
        break;
    case Board_Angle1:
        {
            int16_t angle_cdeg = (int16_t)((CAN_Rx_Mess[0] << 8) | CAN_Rx_Mess[1]);

            float rad = angle_cdeg / 100.0f * 3.1415926f / 180.0f;


            CAN_Send(Board_Angle2, rad, 0, 0, 0);
            break;
        }
    case Board_Angle2:
        {
            board_angle2 = (int16_t)((CAN_Rx_Mess[0] << 8) | CAN_Rx_Mess[1]);
            break;
        }
    default:
        break;
    }
}

/*
 * @brief Init函数中这路CAN配置FIFO0邮箱，使用对应回调函数接收，跳转对应接收函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

   CAN_Receive(&motor2,&motor1);
}
