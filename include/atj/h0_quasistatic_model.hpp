#pragma once

#include <numbers>

namespace atj::h0 {

enum class DifferentialDirection { increasing, decreasing };

struct QuasistaticModelParameters {
    double slope_deg_per_count;
    double bias_deg;
    double direction_bias_deg;
};

[[nodiscard]]
constexpr QuasistaticModelParameters identified_quasistatic_parameters() {
    /*
     * Identified from H0 quasistatic_sweep_01.
     *
     * Empirical branch model:
     *
     * q = a * differential_offset
     *      + b
     *      + h * direction
     *
     * direction:
     *   +1 = increasing differential coordinate
     *   -1 = decreasing differential coordinate
     */
    return {.slope_deg_per_count = 0.2782, .bias_deg = -0.297, .direction_bias_deg = -0.878};
}

class QuasistaticModel {
  public:
    explicit constexpr QuasistaticModel(
        QuasistaticModelParameters parameters = identified_quasistatic_parameters())
        : parameters_(parameters) {}

    [[nodiscard]]
    constexpr double predict_deg(double differential_offset_counts,
                                 DifferentialDirection direction) const {
        const double direction_value = direction == DifferentialDirection::increasing ? 1.0 : -1.0;

        return parameters_.slope_deg_per_count * differential_offset_counts + parameters_.bias_deg +
               parameters_.direction_bias_deg * direction_value;
    }

    [[nodiscard]]
    constexpr double predict_rad(double differential_offset_counts,
                                 DifferentialDirection direction) const {
        return predict_deg(differential_offset_counts, direction) * std::numbers::pi / 180.0;
    }

    [[nodiscard]]
    constexpr QuasistaticModelParameters parameters() const {
        return parameters_;
    }

  private:
    QuasistaticModelParameters parameters_;
};

} // namespace atj::h0
