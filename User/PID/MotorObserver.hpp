#ifndef NEURAL_NETWORK_PID_DEMO_MOTOR_OBSERVER_HPP
#define NEURAL_NETWORK_PID_DEMO_MOTOR_OBSERVER_HPP

#include "RbfNetwork.hpp"

enum class NeuralMode : uint8_t { Off = 0, Learn = 1, Frozen = 2 };

struct NeuralStats {
    NeuralMode mode = NeuralMode::Off;
    bool valid = false;
    float predicted_speed = 0.0f;  // 上一周期已作出的预测，对应本周期实测值。
    float actual_speed = 0.0f;
    float error = 0.0f;           // 实测 - 预测，rpm。
    float mae = 0.0f;
    float baseline_mae = 0.0f;    // 基准：下一时刻速度等于上一时刻速度。
    uint32_t updates = 0;
    uint32_t evaluated = 0;       // 仅在起点有运动/电流激励时计入 MAE。
    uint32_t skipped = 0;
};

// 负责把 t 时刻的输入和 t+2ms 的测量配对，避免用当前指令解释过去的运动。
class MotorObserver {
public:
    void configure(NeuralMode mode, uint32_t revision, uint32_t reset_revision) {
        if (reset_revision != reset_revision_) {
            network_.reset();
            reset_revision_ = reset_revision;
        }
        if (revision != revision_ || reset_revision != stats_reset_revision_) {
            stats_ = {};
            pending_ = false;
            revision_ = revision;
            stats_reset_revision_ = reset_revision;
        }
        mode_ = mode;
        stats_.mode = mode;
        stats_.updates = network_.updates();
    }

    void step(uint32_t now, uint32_t feedback_ms, uint32_t feedback_count,
              float speed, float command, uint32_t control_revision, bool eligible) {
        stats_.valid = false;
        // 网络比控制器的 20ms 失联阈值更严格；旧测量不能充当新的训练答案。
        const bool input_ok = mode_ != NeuralMode::Off && eligible &&
                              feedback_count != 0 && (now - feedback_ms) <= 2 &&
                              network_.supports(speed, command);
        if (pending_) {
            const float delta = speed - previous_speed_;
            const bool pair_ok = input_ok && (now - previous_ms_) == 2 &&
                                 (feedback_ms - previous_feedback_ms_) == 2 &&
                                 feedback_count != previous_feedback_count_ &&
                                 control_revision == previous_control_revision_ &&
                                 std::fabs(delta) <= RbfNetwork::DeltaScale;
            if (pair_ok) {
                // 先评价过去保存的预测，再训练。不能训练完再计算本条预测误差。
                stats_.valid = true;
                stats_.predicted_speed = previous_prediction_;
                stats_.actual_speed = speed;
                stats_.error = speed - previous_prediction_;
                const bool excited = std::fabs(previous_speed_) >= 5.0f ||
                                     std::fabs(previous_command_) >= 200.0f;
                if (excited) {
                    ++stats_.evaluated;
                    const float count = static_cast<float>(stats_.evaluated);
                    stats_.mae += (std::fabs(stats_.error) - stats_.mae) / count;
                    stats_.baseline_mae += (std::fabs(delta) - stats_.baseline_mae) / count;
                }
                if (mode_ == NeuralMode::Learn)
                    network_.learn(previous_speed_, previous_command_, delta);
            } else {
                ++stats_.skipped;
            }
        }
        pending_ = false;

        if (input_ok) {
            previous_ms_ = now;
            previous_feedback_ms_ = feedback_ms;
            previous_feedback_count_ = feedback_count;
            previous_control_revision_ = control_revision;
            previous_speed_ = speed;
            previous_command_ = command;
            previous_prediction_ = speed + network_.predict_delta(speed, command);
            pending_ = true;
        }
        stats_.updates = network_.updates();
    }

    const NeuralStats &stats() const { return stats_; }

private:
    RbfNetwork network_;
    NeuralStats stats_;
    NeuralMode mode_ = NeuralMode::Off;
    uint32_t revision_ = 0;
    uint32_t reset_revision_ = 0;
    uint32_t stats_reset_revision_ = 0;
    bool pending_ = false;
    uint32_t previous_ms_ = 0;
    uint32_t previous_feedback_ms_ = 0;
    uint32_t previous_feedback_count_ = 0;
    uint32_t previous_control_revision_ = 0;
    float previous_speed_ = 0.0f;
    float previous_command_ = 0.0f;
    float previous_prediction_ = 0.0f;
};

#endif
