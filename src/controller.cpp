#include "atj/controller.hpp"

namespace atj {

PositionPController::PositionPController(double kp) : kp_(kp) {}

double PositionPController::differential_command(double q_reference, const State& state) const {
    const double error = q_reference - state.q;

    return kp_ * error;
}

Input make_antagonistic_input(double differential_command, double common_mode) {
    return Input{.theta_1 = common_mode + differential_command / 2.0,
                 .theta_2 = common_mode - differential_command / 2.0};
}

PositionPDController::PositionPDController(double kp, double kd) : kp_(kp), kd_(kd) {}

double PositionPDController::differential_command(double q_reference, const State& state) const {
    const double position_error = q_reference - state.q;

    return kp_ * position_error - kd_ * state.q_dot;
}
} // namespace atj
