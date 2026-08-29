//
// Created by 24707 on 2026/8/8.
//
#include "stm32f4xx.h"
#include <stdint.h>
#ifndef CAN_DEMO_CANIO_H
#define CAN_DEMO_CANIO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t Angle;         // 机械角度 0~8191
    int16_t Speed;         // 转速 (RPM)
    int16_t Current;       // 转矩电流
    uint8_t Temperature;   // 温度
} GM6020_Feedback_t;
extern GM6020_Feedback_t motor2;
extern GM6020_Feedback_t motor1;
void CAN_Init();
void CAN_Send(int16_t ID, int16_t Mess_1, int16_t Mess_2, int16_t Mess_3, int16_t Mess_4);
void CAN_Receive(GM6020_Feedback_t *feedback,GM6020_Feedback_t *feedback2);
#define Motor_2 0x206
#define Motor_1 0x205

#ifdef __cplusplus
}
#endif

#endif //CAN_DEMO_CANIO_H
