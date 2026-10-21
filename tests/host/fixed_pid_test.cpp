// Native logic checks only: the fake HAL does not model motor dynamics or IRQ timing.
#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <vector>
#include "GimbalAxis.hpp"
#include "Debug.hpp"
#include "task.h"

extern "C" void Gimbal_Task(void *);
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
extern "C" {
GM6020_Feedback_t motor1 = {}, motor2 = {}, motor3 = {};
volatile uint32_t motor2_feedback_count = 0, motor2_feedback_ms = 0;
}
UART_HandleTypeDef huart1 = {USART1, nullptr};
static uint8_t *rx_buffer;
static uint32_t now_ms;
static int cycles_left;
static int16_t sent_current;
static std::function<void()> at_delay;
static std::function<void()> at_tick;
static bool can_queue_ok = true;
struct TaskFinished {};

uint32_t HAL_GetTick() {
    if (at_tick) {
        auto action = at_tick;
        at_tick = nullptr;
        action();
    }
    return now_ms;
}
TickType_t xTaskGetTickCount() { return now_ms; }
void vTaskDelayUntil(TickType_t *, TickType_t period) {
    if (at_delay) at_delay();
    now_ms += period;
    if (--cycles_left <= 0) throw TaskFinished{};
}
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *data, uint16_t) {
    rx_buffer = data;
    return HAL_OK;
}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t) { return HAL_OK; }
extern "C" uint8_t CAN_Send(int16_t id, int16_t a, int16_t b, int16_t c, int16_t d) {
    assert(id == 0x1FE && a == 0 && c == 0 && d == 0);
    sent_current = b;
    return can_queue_ok ? 1 : 0;
}
static void command(const char *text) {
    std::memcpy(rx_buffer, text, std::strlen(text));
    HAL_UARTEx_RxEventCallback(&huart1, static_cast<uint16_t>(std::strlen(text)));
}
static void run(int cycles = 1) {
    cycles_left = cycles;
    try { Gimbal_Task(nullptr); } catch (const TaskFinished &) {}
}
static void fresh_feedback() {
    motor2 = {4000, 0, 0, 25};
    motor2_feedback_ms = now_ms;
    ++motor2_feedback_count;
}

int main() {
    uart1_rx_start();
    // Boot and protected mode must reject both movement commands.
    assert(g_protected_mode == 1);
    command("angle=4100");
    command("current=1000");
    run();
    assert(sent_current == 0 && motor2_position_mode == 0);

    fresh_feedback();
    command("Normal Mode");
    command("angle=4100\r\n");
    assert(motor2_position_mode == 1 && motor2_target_angle == 4100);
    run();
    assert(sent_current > 0 && sent_current <= 8000);
    assert(motor2_debug.target_speed_x100 > 0);

    const auto valid_revision = motor2_command_revision;
    for (const char *bad : {"angle=-1", "angle=8192", "angle=", "angle=42x",
                            "current=16385", "current=999999999999999999999999"})
        command(bad);
    assert(motor2_command_revision == valid_revision);

    command("current=-123");
    run();
    assert(motor2_position_mode == 0 && sent_current == -123);
    command("Protected Mode");
    run();
    assert(sent_current == 0 && motor2_current_setpoint == 0);
    command("Normal Mode");
    run();
    assert(sent_current == 0); // Normal must not replay an old current command.

    // Timeout latches STOP and a returning CAN frame cannot restart movement.
    fresh_feedback();
    command("current=500");
    now_ms += 21;
    run();
    assert(sent_current == 0 && g_protected_mode == 1);
    fresh_feedback();
    run();
    assert(sent_current == 0 && g_protected_mode == 1);

    // A stop arriving after the task snapshots a command must cancel that output.
    fresh_feedback();
    command("Normal Mode");
    command("current=500");
    at_tick = [] { command("Protected Mode"); };
    run();
    assert(sent_current == 0);

    // Simulate forward feedback, then stop inside a running task.
    fresh_feedback();
    command("Normal Mode");
    command("angle=4100");
    int cycle = 0;
    at_delay = [&] {
        assert(std::abs(sent_current) <= 8000);
        if (cycle++ == 0) {
            assert(sent_current > 0);
            command("Protected Mode");
        } else {
            assert(sent_current == 0);
        }
    };
    run(3);
    at_delay = nullptr;

    // Shortest-angle wrapping and fractional rpm pass through the real PID.
    g_protected_mode = 0;
    GM6020_Feedback_t fb = {8190, 0, 0, 25};
    Gimbal_Axis axis;
    axis.init(&fb);
    axis.set_target_angle(28);
    assert(axis.Axiscontrol() > 0);
    assert(std::fabs(axis.get_target_speed() - 3.15f) < 0.001f);
    fb.Angle = 28;
    axis.init(&fb);
    axis.set_target_angle(8190);
    assert(axis.Axiscontrol() < 0);
    axis.emergency_stop();
    assert(axis.get_target_speed() == 0.0f);
    fb.Angle = 0;
    fb.Speed = -2000; // Synthetic feedback to stress the updated 8000 output limit.
    axis.init(&fb);
    axis.set_target_angle(4000);
    for (int i = 0; i < 100; ++i)
        assert(std::abs(axis.Axiscontrol()) <= 8000);
    assert(axis.Axiscontrol() == 8000);
    g_protected_mode = 1;
    assert(axis.Axiscontrol() == 0);

    PIDc pid;
    pid.PID_Init(&pid.SpdParam, 20, 0.002f, 0);
    for (int i = 0; i < 10; ++i) pid.SingleLoop_PID(&pid.SpdParam, 0, 10);
    pid.PID_Init(&pid.SpdParam, 20, 0.002f, 0);
    assert(pid.SpdParam.PID_Err_all == 0 && pid.SpdParam.PID_lastout == 0);

    // NN commands never change motion requests, even when accepted in STOP.
    const uint32_t control_revision = motor2_command_revision;
    command("nn=learn");
    assert(motor2_neural_mode == NeuralMode::Learn && g_protected_mode == 1);
    assert(motor2_command_revision == control_revision);
    command("nn=freeze");
    assert(motor2_neural_mode == NeuralMode::Frozen);
    command("nn=off");
    assert(motor2_neural_mode == NeuralMode::Off);
    command("nn=reset");
    const uint32_t neural_revision = motor2_neural_revision;
    command("nn=unknown");
    assert(motor2_neural_revision == neural_revision);

    // Identical feedback/targets must produce exactly identical motor commands
    // with NN disabled and learning. This tests the real task integration.
    auto trace = [&](bool learning, bool queued) {
        command("nn=reset");
        if (learning) command("nn=learn");
        fresh_feedback();
        motor2.Speed = 20;
        command("Normal Mode");
        command("angle=4200");
        can_queue_ok = queued;
        std::vector<int16_t> outputs;
        unsigned sample = 0;
        at_delay = [&] {
            outputs.push_back(sent_current);
            motor2.Speed = static_cast<int16_t>(20 + (++sample % 5));
            motor2.Angle = static_cast<int16_t>(4000 + sample);
            motor2_feedback_ms = now_ms + 2;
            ++motor2_feedback_count;
        };
        run(30);
        at_delay = nullptr;
        return outputs;
    };
    const auto disabled_outputs = trace(false, true);
    const auto learning_outputs = trace(true, true);
    assert(disabled_outputs == learning_outputs);
    assert(motor2_debug.neural.updates == 29 && motor2_debug.neural.valid);
    trace(true, false);
    assert(motor2_debug.neural.updates == 0 && !motor2_debug.neural.valid);
    can_queue_ok = true;
    std::cout << "PASS: command validation, mode changes, STOP, feedback timeout, "
                 "stale-command cancellation, angle wrap, float cascade, PID reset, "
                 "NN preserves PID outputs, failed sends excluded\n";
}
