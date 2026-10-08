#include "atj/h0_coordinates.hpp"

#include "atj/h0_calibration.hpp"

#include <stdexcept>

namespace atj::h0 {

MotorWinding motor_winding_from_count_deltas(const MotorCountDelta& count_delta) {
    return MotorWinding{.motor_1_m = winding_displacement_m_from_count_delta(count_delta.motor_1),
                        .motor_2_m = winding_displacement_m_from_count_delta(count_delta.motor_2)};
}

WindingCoordinates winding_coordinates_from_motor_winding(const MotorWinding& winding) {
    return WindingCoordinates{.common_m = (winding.motor_1_m + winding.motor_2_m) / 2.0,
                              .differential_m = winding.motor_2_m - winding.motor_1_m};
}

MotorWinding motor_winding_from_coordinates(const WindingCoordinates& coordinates) {
    return MotorWinding{.motor_1_m = coordinates.common_m - coordinates.differential_m / 2.0,

                        .motor_2_m = coordinates.common_m + coordinates.differential_m / 2.0};
}

Input model_input_from_motor_winding(const MotorWinding& winding, double motor_radius_m) {
    if (motor_radius_m <= 0.0) {
        throw std::invalid_argument("Motor radius must be positive");
    }

    /*
     *  Physical Motor 2 is Model-v0 tendon 1 (+q).
     *  Physical Motor 1 is Model-v0 tendon 2 (-q).
     */
    return Input{.theta_1 = winding.motor_2_m / motor_radius_m,

                 .theta_2 = winding.motor_1_m / motor_radius_m};
}

} // namespace atj::h0
