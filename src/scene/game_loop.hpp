// Deterministic fixed-timestep loop. Simulation always advances in whole
// `step` increments regardless of how real time is sliced; leftover time is
// exposed as an interpolation factor for rendering.
#pragma once
#include <cstdint>
#include <functional>

namespace ifg {

class FixedStepLoop {
public:
    explicit FixedStepLoop(double step, int max_steps_per_advance = 8) : step_(step), max_steps_(max_steps_per_advance) {}

    std::function<void(double step, std::uint64_t tick)> on_fixed_update;

    // Feed elapsed real time. Returns the number of fixed steps executed.
    int advance(double real_dt) {
        accumulator_ += real_dt;
        int n = 0;
        while (accumulator_ >= step_ - 1e-12) {
            if (n == max_steps_) {  // spiral-of-death guard: drop the backlog
                dropped_ += accumulator_;
                accumulator_ = 0;
                break;
            }
            if (on_fixed_update) on_fixed_update(step_, tick_);
            ++tick_;
            ++n;
            accumulator_ -= step_;
        }
        if (accumulator_ < 0) accumulator_ = 0;
        return n;
    }
    double alpha() const { return accumulator_ / step_; }
    std::uint64_t tick() const { return tick_; }
    double sim_time() const { return static_cast<double>(tick_) * step_; }
    double dropped_time() const { return dropped_; }
    double step() const { return step_; }

private:
    double step_;
    int max_steps_;
    double accumulator_ = 0, dropped_ = 0;
    std::uint64_t tick_ = 0;
};

}  // namespace ifg
