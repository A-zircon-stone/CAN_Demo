//
// Created by 24707 on 2026/8/27.
//

#ifndef CAN_DEMO_DEBUG_HPP
#define CAN_DEMO_DEBUG_HPP

#include "usart.h"
#include <cstdarg>
#include <string>

void usart_printf(const char *format, ...);
void uart1_rx_start(void);



#endif //CAN_DEMO_DEBUG_HPP
