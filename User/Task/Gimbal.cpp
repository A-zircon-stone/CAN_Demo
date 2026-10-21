#include "FreeRTOS.h"
#include "task.h"
#include "canio.h"
#include "GimbalAxis.hpp"
#include "Debug.hpp"

Gimbal_Axis yaw;
Gimbal_Axis pitch;
volatile int16_t motor2_current_setpoint = 0;
volatile uint8_t motor2_position_mode = 0;
volatile int16_t motor2_target_angle = 0;
volatile uint32_t motor2_command_revision = 0;
volatile NeuralMode motor2_neural_mode = NeuralMode::Off;
volatile uint32_t motor2_neural_revision = 0;
volatile uint32_t motor2_neural_reset_revision = 0;
GimbalDebugSnapshot motor2_debug = {};

/* 固定 PID 双环 + 只读的 RBF 观察器，每 2ms 更新二号电机。 */
extern "C" void Gimbal_Task(void *argument)
{
    (void)argument;
    GM6020_Feedback_t axis_feedback = {};
    yaw.init(&axis_feedback);
    MotorObserver observer;
    uint32_t last_revision = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    uart1_rx_start();

    for (;;)
    {
        taskENTER_CRITICAL();
        // CAN 中断和串口中断都只更新数据，PID 只在本任务中运行。
        axis_feedback = motor2;
        const uint32_t feedback_count = motor2_feedback_count;
        const uint32_t feedback_ms = motor2_feedback_ms;
        const uint32_t revision = motor2_command_revision;
        const bool protected_mode = g_protected_mode != 0;
        const bool position_mode = motor2_position_mode != 0;
        const int16_t target = motor2_target_angle;
        const int16_t manual_current = motor2_current_setpoint;
        const NeuralMode neural_mode = motor2_neural_mode;
        const uint32_t neural_revision = motor2_neural_revision;
        const uint32_t neural_reset_revision = motor2_neural_reset_revision;
        taskEXIT_CRITICAL();

        const uint32_t now = HAL_GetTick();
        const bool fresh = feedback_count != 0 && (now - feedback_ms) <= 20;
        int16_t output = 0;
        if (revision != last_revision || protected_mode || !fresh)
        {
            yaw.emergency_stop();
            last_revision = revision;
        }

        if (!protected_mode && fresh)
        {
            if (position_mode)
            {
                // 使用单圈最短角误差；不设置机械角度限位。
                yaw.set_target_angle(target);
                output = yaw.Axiscontrol();
            }
            else
            {
                output = manual_current;
            }
        }

        taskENTER_CRITICAL();
        if (!fresh && !g_protected_mode && motor2_command_revision == revision)
        {
            // 反馈丢失后不自动恢复运动，需要重新发送 Normal Mode。
            g_protected_mode = 1;
            motor2_position_mode = 0;
            motor2_current_setpoint = 0;
            ++motor2_command_revision;
        }
        // 计算期间收到停止或新命令时，丢弃本次旧指令。
        if (g_protected_mode || motor2_command_revision != revision)
            output = 0;
        const bool command_queued = CAN_Send(0x1FE, 0, output, 0, 0) != 0;
        const bool observe = command_queued && fresh && position_mode &&
                             !g_protected_mode && motor2_command_revision == revision &&
                             motor2_neural_revision == neural_revision;
        GimbalDebugSnapshot debug = {
            now, axis_feedback.Angle, axis_feedback.Speed, axis_feedback.Current,
            target, output, static_cast<int32_t>(yaw.get_target_speed() * 100.0f),
            axis_feedback.Temperature, g_protected_mode,
            static_cast<uint8_t>(position_mode), static_cast<uint8_t>(fresh), {}
        };
        taskEXIT_CRITICAL();

        // 电流已经由固定 PID 决定并排队。网络运算在临界区外，不回写控制器。
        observer.configure(neural_mode, neural_revision, neural_reset_revision);
        observer.step(now, feedback_ms, feedback_count, axis_feedback.Speed,
                      output, revision, observe);
        debug.neural = observer.stats();
        taskENTER_CRITICAL();
        motor2_debug = debug;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2));
    }
}
