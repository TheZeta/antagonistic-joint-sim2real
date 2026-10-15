#pragma once

#include "atj/h0_quasistatic_model.hpp"

#include <cmath>
#include <numbers>

namespace atj::h0 {

struct DynamicModelParameters {
    double relaxation_time_s;
};

[[nodiscard]]
constexpr DynamicModelParameters identified_dynamic_parameters() {
    return {/*
             * Median first-order relaxation time identified
             * from relaxation probe experiment.
             */
            .relaxation_time_s = 0.584};
}

class DynamicQuasistaticModel {
  public:
    explicit DynamicQuasistaticModel(
        QuasistaticModel quasistatic_model = QuasistaticModel{},
        DynamicModelParameters parameters = identified_dynamic_parameters())
        : quasistatic_model_(quasistatic_model), parameters_(parameters) {}

    void reset_deg(double q_deg) {
        q_deg_ = q_deg;
        initialized_ = true;
    }

    void reset_rad(double q_rad) {
        reset_deg(q_rad * 180.0 / std::numbers::pi);
    }

    [[nodiscard]]
    double update_deg(double dt_s, double differential_offset_counts,
                      DifferentialDirection direction) {
        const double equilibrium_deg =
            quasistatic_model_.predict_deg(differential_offset_counts, direction);

        if (!initialized_) {
            q_deg_ = equilibrium_deg;
            initialized_ = true;

            return q_deg_;
        }

        if (dt_s <= 0.0) {
            return q_deg_;
        }

        const double tau = parameters_.relaxation_time_s;
        const double decay = std::exp(-dt_s / tau);

        q_deg_ = equilibrium_deg + (q_deg_ - equilibrium_deg) * decay;

        return q_deg_;
    }

    [[nodiscard]]
    double update_rad(double dt_s, double differential_offset_counts,
                      DifferentialDirection direction) {
        return update_deg(dt_s, differential_offset_counts, direction) * std::numbers::pi / 180.0;
    }

    [[nodiscard]]
    double q_deg() const {
        return q_deg_;
    }

    [[nodiscard]]
    double q_rad() const {
        return q_deg_ * std::numbers::pi / 180.0;
    }

    [[nodiscard]]
    DynamicModelParameters parameters() const {
        return parameters_;
    }

  private:
    QuasistaticModel quasistatic_model_;
    DynamicModelParameters parameters_;

    bool initialized_ = false;
    double q_deg_ = 0.0;
};

} // namespace atj::h0
