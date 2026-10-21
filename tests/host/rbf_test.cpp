#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "MotorObserver.hpp"

// A deterministic synthetic response, not a GM6020 model or hardware result.
static float plant_delta(float speed, float command) {
    return 0.0006f * command - 0.01f * speed;
}
static float validation_mse(const RbfNetwork &network) {
    float sum = 0;
    unsigned n = 0;
    for (float speed : {-200.0f, -40.0f, 90.0f, 230.0f})
        for (float command : {-4000.0f, -900.0f, 1300.0f, 4600.0f}) {
            const float e = network.predict_delta(speed, command) - plant_delta(speed, command);
            sum += e * e;
            ++n;
        }
    return sum / static_cast<float>(n);
}

int main() {
    RbfNetwork network;
    const float before = validation_mse(network);
    for (int epoch = 0; epoch < 1000; ++epoch)
        for (float speed : {-250.0f, -125.0f, 0.0f, 125.0f, 250.0f})
            for (float command : {-5000.0f, -2500.0f, 0.0f, 2500.0f, 5000.0f})
                assert(network.learn(speed, command, plant_delta(speed, command)));
    const float after = validation_mse(network);
    assert(after < before * 0.1f); // Check on inputs not used for training.
    const auto learned = network.updates();
    assert(!network.learn(513, 0, 1));
    assert(!network.learn(0, 8001, 1));
    assert(!network.learn(0, 0, 33));
    assert(!network.learn(0, 0, std::numeric_limits<float>::quiet_NaN()));
    assert(network.updates() == learned);
    network.reset();
    assert(network.updates() == 0 && network.predict_delta(100, 1000) == 0);

    MotorObserver observer;
    observer.configure(NeuralMode::Learn, 1, 0);
    observer.step(0, 0, 1, 10, 1000, 7, true);
    assert(!observer.stats().valid && observer.stats().updates == 0);
    observer.step(2, 2, 2, 12, 1000, 7, true);
    // Prediction must have been made before seeing 12, using the zeroed model.
    assert(observer.stats().valid && observer.stats().updates == 1);
    assert(observer.stats().predicted_speed == 10 && observer.stats().actual_speed == 12);
    assert(observer.stats().mae == 2 && observer.stats().baseline_mae == 2);

    observer.configure(NeuralMode::Frozen, 2, 0);
    assert(observer.stats().evaluated == 0 && observer.stats().updates == 1);
    observer.step(4, 4, 3, 10, 1000, 7, true);
    observer.step(6, 6, 4, 12, 1000, 7, true);
    assert(observer.stats().valid && observer.stats().updates == 1);
    assert(observer.stats().predicted_speed > 10); // Learned weights survived freeze.
    observer.step(8, 6, 4, 12, 1000, 7, true); // Repeated CAN feedback.
    assert(!observer.stats().valid && observer.stats().updates == 1);
    observer.step(12, 12, 5, 13, 1000, 7, true); // Missed control period.
    assert(!observer.stats().valid && observer.stats().skipped == 2);
    observer.step(14, 14, 6, 13, 1000, 8, true); // New motion request.
    assert(!observer.stats().valid && observer.stats().skipped == 3);
    observer.step(16, 16, 7, 14, 1000, 8, false); // STOP or failed send.
    assert(!observer.stats().valid && observer.stats().updates == 1);

    observer.configure(NeuralMode::Off, 3, 0);
    observer.step(18, 18, 8, 14, 1000, 8, true);
    observer.step(20, 20, 9, 15, 1000, 8, true);
    assert(!observer.stats().valid && observer.stats().updates == 1);

    observer.configure(NeuralMode::Learn, 4, 1);
    assert(observer.stats().updates == 0);
    observer.step(0xFFFFFFFEu, 0xFFFFFFFEu, 1, 10, 1000, 7, true);
    observer.step(0, 0, 2, 12, 1000, 7, true); // Millisecond counter wrap.
    assert(observer.stats().valid && observer.stats().updates == 1);
    observer.step(2, 2, 3, 60, 1000, 7, true); // Implausible one-step change.
    assert(!observer.stats().valid && observer.stats().updates == 1);
    std::cout << "PASS: held-out synthetic MSE " << before << " -> " << after
              << "; causal scoring, normalization, freeze/off/reset, timing gaps, "
                 "repeated feedback, failed input, wrap-around\n";
}
