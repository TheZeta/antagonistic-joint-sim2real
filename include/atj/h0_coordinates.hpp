#pragma once

#include "atj/joint_model.hpp"

namespace atj::h0 {

struct MotorCountDelta {
    int motor_1;
    int motor_2;
};

struct MotorWinding {
    /*
     * Positive -> WIND
     * Negative -> UNWIND
     */
    double motor_1_m;
    double motor_2_m;
};

struct WindingCoordinates {
    /*
     * Positive common mode -> both motors WIND.
     */
    double common_m;

    /*
     * Positive differential mode -> q+ actuation.
     *
     * Defined as:
     *
     *      differential = w2 - w1
     */
    double differential_m;
};

[[nodiscard]]
MotorWinding motor_winding_from_count_deltas(const MotorCountDelta& count_delta);

[[nodiscard]]
WindingCoordinates winding_coordinates_from_motor_winding(const MotorWinding& winding);

[[nodiscard]]
MotorWinding motor_winding_from_coordinates(const WindingCoordinates& coordinates);

/*
 * Bridge from realized H0 motors to Model v0.
 *
 * Model v0 convention:
 *
 *      theta_1 -> positive-q tendon
 *      theta_2 -> negative-q tendon
 *
 * Real H0:
 *
 *      Motor 2 WIND -> q+
 *      Motor 1 WIND -> q-
 *
 * Therefore:
 *
 *      Model theta_1 <-> physical Motor 2
 *      Model theta_2 <-> physical Motor 1
 */
[[nodiscard]]
Input model_input_from_motor_winding(const MotorWinding& winding, double motor_radius_m);

} // namespace atj::h0
