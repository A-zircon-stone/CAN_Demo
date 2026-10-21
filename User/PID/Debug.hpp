//
// Created by 24707 on 2026/8/27.
//

#ifndef CAN_DEMO_DEBUG_HPP
#define CAN_DEMO_DEBUG_HPP

#include "usart.h"
#include <cstdarg>
#include <string>
#include "MotorObserver.hpp"

void usart_printf(const char *format, ...);
void uart1_rx_start(void);
extern volatile int16_t motor2_current_setpoint;
extern volatile uint8_t g_protected_mode;
extern volatile uint8_t motor2_position_mode;
extern volatile int16_t motor2_target_angle;
extern volatile uint32_t motor2_command_revision;
extern volatile NeuralMode motor2_neural_mode;
extern volatile uint32_t motor2_neural_revision;
extern volatile uint32_t motor2_neural_reset_revision;

// 控制任务发布的同一周期快照，打印任务在临界区内复制。
struct GimbalDebugSnapshot {
    uint32_t time_ms;
    int16_t angle;
    int16_t speed;
    int16_t measured_current;
    int16_t target_angle;
    int16_t output_current;
    int32_t target_speed_x100;
    uint8_t temperature;
    uint8_t protected_mode;
    uint8_t position_mode;
    uint8_t feedback_fresh;
    NeuralStats neural;
};
extern GimbalDebugSnapshot motor2_debug;



#endif //CAN_DEMO_DEBUG_HPP
