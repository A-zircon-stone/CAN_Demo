//
// Created by 24707 on 2026/8/30.
//

#ifndef CAN_DEMO_BOARD_HPP
#define CAN_DEMO_BOARD_HPP

#include <stdint.h>
#define BoardHello  0x305
#define BoardRespect  0x306

#define Board_Angle1  0x301//B板
#define Board_Angle2  0x302//A板
#define Board_Angle3  0x303//A板->B板，弧度请求
#define Board_Angle4  0x304//B板->A板，角度回包
extern volatile uint8_t board_link_ok;
#define BOARD_ROLE_MASTER 1
#define BOARD_ROLE_SLAVE  2
#define BOARD_ROLE BOARD_ROLE_MASTER//这是A板
//#define BOARD_ROLE BOARD_ROLE_SLAVE//B板

#endif //CAN_DEMO_BOARD_HPP
