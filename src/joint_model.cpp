#include "atj/joint_model.hpp"

namespace atj {

JointModel::JointModel(Parameters parameters) : parameters_(parameters) {}

double JointModel::tendon_1_extension(const State& state, const Input& input) const {
    return parameters_.preload + parameters_.motor_radius * input.theta_1 -
           parameters_.joint_radius * state.q;
}

double JointModel::tendon_2_extension(const State& state, const Input& input) const {
    return parameters_.preload + parameters_.motor_radius * input.theta_2 +
           parameters_.joint_radius * state.q;
}

double JointModel::tendon_1_force(const State& state, const Input& input) const {
    const double extension = tendon_1_extension(state, input);

    return parameters_.spring_stiffness * extension;
}

double JointModel::tendon_2_force(const State& state, const Input& input) const {
    const double extension = tendon_2_extension(state, input);

    return parameters_.spring_stiffness * extension;
}

StateDerivative JointModel::derivative(const State& state, const Input& input) const {
    const double force_1 = tendon_1_force(state, input);
    const double force_2 = tendon_2_force(state, input);

    const double tendon_torque = parameters_.joint_radius * (force_1 - force_2);

    const double damping_torque = -parameters_.damping * state.q_dot;

    const double net_torque = tendon_torque + damping_torque;

    const double q_ddot = net_torque / parameters_.inertia;

    return StateDerivative{.q_dot = state.q_dot, .q_ddot = q_ddot};
}

} // namespace atj
