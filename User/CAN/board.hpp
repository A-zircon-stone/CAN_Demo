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
extern volatile uint8_t board_link_ok;
#define BOARD_ROLE_MASTER 1
#define BOARD_ROLE_SLAVE  2
#define BOARD_ROLE BOARD_ROLE_MASTER//这是A板
//#define BOARD_ROLE BOARD_ROLE_SLAVE//B板

#endif //CAN_DEMO_BOARD_HPP
