#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace atj::h0 {

struct ModelV1Parameters {
    double slope_deg_per_count;
    double bias_deg;
    double memory_gain_deg;
    double memory_transition_counts;
    double relaxation_time_s;
};

[[nodiscard]]
constexpr ModelV1Parameters identified_model_v1_parameters() {
    return {.slope_deg_per_count = 0.2821,
            .bias_deg = -0.284,
            .memory_gain_deg = -0.956,
            .memory_transition_counts = 3.310,
            .relaxation_time_s = 0.59};
}

class ModelV1 {
  public:
    explicit ModelV1(ModelV1Parameters parameters = identified_model_v1_parameters())
        : parameters_(parameters) {}

    /*
     * Generic explicit initialization.
     *
     * memory_state must lie in [-1,+1].
     */
    void reset(double differential_offset_counts, double q_deg, double memory_state) {
        previous_offset_counts_ = differential_offset_counts;
        q_deg_ = q_deg;
        memory_state_ = std::clamp(memory_state, -1.0, 1.0);
        initialized_ = true;
    }

    /*
     * Useful when q is believed to already be at
     * quasi-static equilibrium.
     *
     * Infer z from:
     *
     * q = a*x + b + h*z
     */
    void reset_from_equilibrium(double differential_offset_counts, double q_deg) {
        double memory_state = 0.0;
        if (std::abs(parameters_.memory_gain_deg) > 0.0) {
            memory_state = (q_deg - parameters_.slope_deg_per_count * differential_offset_counts -
                            parameters_.bias_deg) /
                           parameters_.memory_gain_deg;
        }

        reset(differential_offset_counts, q_deg, memory_state);
    }

    [[nodiscard]]
    double update_deg(double dt_s, double differential_offset_counts) {
        if (!initialized_) {
            previous_offset_counts_ = differential_offset_counts;
            memory_state_ = 0.0;
            q_deg_ = equilibrium_deg(differential_offset_counts);
            initialized_ = true;

            return q_deg_;
        }

        update_memory(differential_offset_counts);
        const double equilibrium = equilibrium_deg(differential_offset_counts);

        if (dt_s <= 0.0) {
            return q_deg_;
        }

        const double decay = std::exp(-dt_s / parameters_.relaxation_time_s);
        q_deg_ = equilibrium + (q_deg_ - equilibrium) * decay;

        return q_deg_;
    }

    [[nodiscard]]
    double update_rad(double dt_s, double differential_offset_counts) {
        return update_deg(dt_s, differential_offset_counts) * std::numbers::pi / 180.0;
    }

    [[nodiscard]]
    double equilibrium_deg(double differential_offset_counts) const {
        return parameters_.slope_deg_per_count * differential_offset_counts + parameters_.bias_deg +
               parameters_.memory_gain_deg * memory_state_;
    }

    [[nodiscard]]
    double memory_state() const {
        return memory_state_;
    }

    [[nodiscard]]
    double q_deg() const {
        return q_deg_;
    }

    [[nodiscard]]
    ModelV1Parameters parameters() const {
        return parameters_;
    }

  private:
    void update_memory(double differential_offset_counts) {
        const double delta = differential_offset_counts - previous_offset_counts_;

        if (delta == 0.0) {
            return;
        }

        const double direction = delta > 0.0 ? 1.0 : -1.0;
        const double decay = std::exp(-std::abs(delta) / parameters_.memory_transition_counts);

        memory_state_ = direction + (memory_state_ - direction) * decay;
        previous_offset_counts_ = differential_offset_counts;
    }

    ModelV1Parameters parameters_;
    bool initialized_ = false;
    double previous_offset_counts_ = 0.0;
    double memory_state_ = 0.0;
    double q_deg_ = 0.0;
};

} // namespace atj::h0
