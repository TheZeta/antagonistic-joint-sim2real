#pragma once

namespace atj {

struct Parameters {
    double inertia;
    double damping;
    double spring_stiffness;
    double joint_radius;
    double motor_radius;
    double preload;
};

struct State {
    double q;
    double q_dot;
};

struct Input {
    double theta_1;
    double theta_2;
};

struct StateDerivative {
    double q_dot;
    double q_ddot;
};

class JointModel {
  public:
    explicit JointModel(Parameters parameters);

    [[nodiscard]]
    StateDerivative derivative(const State& state, const Input& input) const;

    [[nodiscard]]
    double tendon_1_extension(const State& state, const Input& input) const;

    [[nodiscard]]
    double tendon_2_extension(const State& state, const Input& input) const;

    [[nodiscard]]
    double tendon_1_force(const State& state, const Input& input) const;

    [[nodiscard]]
    double tendon_2_force(const State& state, const Input& input) const;

  private:
    Parameters parameters_;
};

} // namespace atj
