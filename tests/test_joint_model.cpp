#include "atj/joint_model.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

constexpr double tolerance = 1e-9;

bool near(double actual, double expected, double tol = tolerance) {
    return std::abs(actual - expected) <= tol;
}

void expect_near(double actual, double expected, const std::string& description) {
    if (!near(actual, expected)) {
        std::cerr << "FAILED: " << description << '\n'
                  << " expected:  " << expected << '\n'
                  << " actual:    " << actual << '\n';

        std::exit(EXIT_FAILURE);
    }
}

atj::Parameters default_parameters() {
    return atj::Parameters{.inertia = 0.01,
                           .damping = 0.02,
                           .spring_stiffness = 200.0,
                           .joint_radius = 0.02,
                           .motor_radius = 0.01,
                           .preload = 0.01};
}

void test_initial_preload() {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 0.0, .theta_2 = 0.0};

    expect_near(model.tendon_1_extension(state, input), 0.01, "initial tendon 1 extension");

    expect_near(model.tendon_2_extension(state, input), 0.01, "initial tendon 2 extension");

    expect_near(model.tendon_1_force(state, input), 2.0, "initial tendon 1 force");

    expect_near(model.tendon_2_force(state, input), 2.0, "initial tendon 2 force");

    const auto derivative = model.derivative(state, input);

    expect_near(derivative.q_ddot, 0.0, "initial joint acceleration");
}

void test_differential_actuation() {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    expect_near(model.tendon_1_extension(state, input), 0.02, "differential tendon 1 extension");

    expect_near(model.tendon_2_extension(state, input), 0.01, "differential tendon 2 extension");

    expect_near(model.tendon_1_force(state, input), 4.0, "differential tendon 1 force");

    expect_near(model.tendon_2_force(state, input), 2.0, "differential tendon 2 force");

    const auto derivative = model.derivative(state, input);

    expect_near(derivative.q_ddot, 4.0, "differential joint acceleration");
}

void test_static_equilibrium() {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.25, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    expect_near(model.tendon_1_force(state, input), 3.0, "equilibrium tendon 1 force");

    expect_near(model.tendon_2_force(state, input), 3.0, "equilibrium tendon 2 force");

    const auto derivative = model.derivative(state, input);

    expect_near(derivative.q_ddot, 0.0, "equilibrium joint acceleration");
}

void test_common_mode_actuation() {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 1.0};

    expect_near(model.tendon_1_extension(state, input), 0.02, "common-mode tendon 1 extension");

    expect_near(model.tendon_2_extension(state, input), 0.02, "common-mode tendon 2 extension");

    expect_near(model.tendon_1_force(state, input), 4.0, "common-mode tendon 1 force");

    expect_near(model.tendon_2_force(state, input), 4.0, "common-mode tendon 2 force");

    const auto derivative = model.derivative(state, input);

    expect_near(derivative.q_ddot, 0.0, "common-mode joint acceleration");
}

} // namespace

int main() {
    test_initial_preload();
    test_differential_actuation();
    test_static_equilibrium();
    test_common_mode_actuation();

    std::cout << "All joint model tests passed.\n";

    return EXIT_SUCCESS;
}
