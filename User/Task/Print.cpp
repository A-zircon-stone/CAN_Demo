#include "FreeRTOS.h"
#include "task.h"
#include "Debug.hpp"
#include "canio.h"

/* 25Hz 显示控制与预测快照，给 115200 波特率预留带宽。 */
extern "C" void Printf_Task(void *argument)
{
    (void)argument;
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        taskENTER_CRITICAL();
        const GimbalDebugSnapshot state = motor2_debug;
        taskEXIT_CRITICAL();

        const char *mode = state.protected_mode ? "STOP" :
                           (state.position_mode ? "POS" : "CURRENT");
        const char *neural_mode = state.neural.mode == NeuralMode::Learn ? "LEARN" :
                                  (state.neural.mode == NeuralMode::Frozen ? "FROZEN" : "OFF");
        usart_printf("M2 ms=%lu Mode=%s Fresh=%u Target=%d Angle=%d Vref100=%ld Speed=%d Out=%d Current=%d Temp=%u NN=%s Valid=%u Pred100=%ld Obs100=%ld Err100=%ld MAE100=%ld Base100=%ld Eval=%lu N=%lu Skip=%lu\r\n",
                     static_cast<unsigned long>(state.time_ms), mode,
                     static_cast<unsigned int>(state.feedback_fresh),
                     static_cast<int>(state.target_angle), static_cast<int>(state.angle),
                     static_cast<long>(state.target_speed_x100), static_cast<int>(state.speed),
                     static_cast<int>(state.output_current), static_cast<int>(state.measured_current),
                     static_cast<unsigned int>(state.temperature), neural_mode,
                     static_cast<unsigned int>(state.neural.valid),
                     static_cast<long>(state.neural.predicted_speed * 100.0f),
                     static_cast<long>(state.neural.actual_speed * 100.0f),
                     static_cast<long>(state.neural.error * 100.0f),
                     static_cast<long>(state.neural.mae * 100.0f),
                     static_cast<long>(state.neural.baseline_mae * 100.0f),
                     static_cast<unsigned long>(state.neural.evaluated),
                     static_cast<unsigned long>(state.neural.updates),
                     static_cast<unsigned long>(state.neural.skipped));

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(40));
    }
}
