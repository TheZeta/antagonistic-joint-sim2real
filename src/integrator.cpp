#include "atj/integrator.hpp"

namespace atj {

namespace {

State add_scaled_derivative(const State& state, const StateDerivative& derivative, double scale) {
    return State{.q = state.q + scale * derivative.q_dot,
                 .q_dot = state.q_dot + scale * derivative.q_ddot};
}

} // namespace

State euler_step(const JointModel& model, const State& state, const Input& input, double dt) {
    const StateDerivative derivative = model.derivative(state, input);

    return State{.q = state.q + dt * derivative.q_dot,
                 .q_dot = state.q_dot + dt * derivative.q_ddot};
}

State rk4_step(const JointModel& model, const State& state, const Input& input, double dt) {
    const StateDerivative k1 = model.derivative(state, input);

    const State state_k2 = add_scaled_derivative(state, k1, dt / 2.0);
    const StateDerivative k2 = model.derivative(state_k2, input);

    const State state_k3 = add_scaled_derivative(state, k2, dt / 2.0);
    const StateDerivative k3 = model.derivative(state_k3, input);

    const State state_k4 = add_scaled_derivative(state, k3, dt);
    const StateDerivative k4 = model.derivative(state_k4, input);

    return State{.q =
                     state.q + (dt / 6.0) * (k1.q_dot + 2.0 * k2.q_dot + 2.0 * k3.q_dot + k4.q_dot),
                 .q_dot = state.q_dot +
                          (dt / 6.0) * (k1.q_ddot + 2.0 * k2.q_ddot + 2.0 * k3.q_ddot + k4.q_ddot)};
}

} // namespace atj
