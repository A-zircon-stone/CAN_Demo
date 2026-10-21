#ifndef NEURAL_NETWORK_PID_DEMO_RBF_NETWORK_HPP
#define NEURAL_NETWORK_PID_DEMO_RBF_NETWORK_HPP

#include <cmath>
#include <cstdint>

// 第二步：只学习电机响应。输入使用物理单位，类内部完成归一化。
// 输入 [当前 rpm, 已排队发送的电流指令]，输出下一个 2ms 的转速变化 rpm。
// 9 个固定高斯节点，仅学习输出权重；不持有也不修改任何 PID 参数。
class RbfNetwork {
public:
    static constexpr unsigned NodeCount = 9;
    static constexpr float SpeedScale = 512.0f;  // 归一化范围，并非电机限速。
    static constexpr float CommandScale = 8000.0f;
    static constexpr float DeltaScale = 32.0f;

    void reset() {
        for (float &weight : weights_) weight = 0.0f;
        updates_ = 0;
    }

    bool supports(float speed, float command) const {
        return std::isfinite(speed) && std::isfinite(command) &&
               std::fabs(speed) <= SpeedScale && std::fabs(command) <= CommandScale;
    }

    float predict_delta(float speed, float command) const {
        if (!supports(speed, command)) return 0.0f;
        float output = 0.0f;
        for (unsigned j = 0; j < NodeCount; ++j)
            output += weights_[j] * basis(j, speed, command);
        return bounded(output, 1.0f) * DeltaScale;
    }

    bool learn(float speed, float command, float actual_delta) {
        if (!supports(speed, command) || !std::isfinite(actual_delta) ||
            std::fabs(actual_delta) > DeltaScale)
            return false;

        float h[NodeCount];
        float output = 0.0f;
        float norm = 0.001f;
        for (unsigned j = 0; j < NodeCount; ++j) {
            h[j] = basis(j, speed, command);
            output += weights_[j] * h[j];
            norm += h[j] * h[j];
        }
        const float error = actual_delta / DeltaScale - output;
        for (unsigned j = 0; j < NodeCount; ++j) {
            // 归一化 LMS。初版固定中心和宽度，学习过程更容易核对。
            weights_[j] = bounded(weights_[j] + 0.02f * error * h[j] / norm, 2.0f);
        }
        ++updates_;
        return true;
    }

    uint32_t updates() const { return updates_; }

private:
    static float bounded(float value, float limit) {
        return value > limit ? limit : (value < -limit ? -limit : value);
    }
    static float basis(unsigned j, float speed, float command) {
        // 中心是 [-1, 0, 1] × [-1, 0, 1] 的九宫格，宽度为 0.8。
        const float speed_center = static_cast<float>(j / 3) - 1.0f;
        const float command_center = static_cast<float>(j % 3) - 1.0f;
        const float a = speed / SpeedScale - speed_center;
        const float b = command / CommandScale - command_center;
        return expf(-(a * a + b * b) / (2.0f * 0.8f * 0.8f));
    }

    float weights_[NodeCount] = {};
    uint32_t updates_ = 0;
};

#endif
