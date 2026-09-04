#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

constexpr double tolerance = 1e-10;

bool near(double actual, double expected, double tol = tolerance) {
    return std::abs(actual - expected) <= tol;
}

void expect_near(double actual, double expected, double tolerance_value,
                 const std::string& description) {
    if (!near(actual, expected, tolerance_value)) {
        std::cerr << std::setprecision(15);
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

void test_euler_first_step() {
    const atj::JointModel model(default_parameters());

    const atj::State initial_state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.01;

    const atj::State result = atj::euler_step(model, initial_state, input, dt);

    expect_near(result.q, 0.0, tolerance, "Euler first-step position");

    expect_near(result.q_dot, 0.04, tolerance, "Euler first-step velocity");
}

void test_equilibrium_remains_equilibrium() {
    const atj::JointModel model(default_parameters());

    const atj::State equilibrium{.q = 0.25, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.01;

    const atj::State euler_result = atj::euler_step(model, equilibrium, input, dt);

    const atj::State rk4_result = atj::rk4_step(model, equilibrium, input, dt);

    expect_near(euler_result.q, 0.25, tolerance, "Euler equilibrium position");

    expect_near(euler_result.q_dot, 0.0, tolerance, "Euler equilibrium velocity");

    expect_near(rk4_result.q, 0.25, tolerance, "RK4 equilibrium position");

    expect_near(rk4_result.q_dot, 0.0, tolerance, "RK4 equilibrium velocity");
}

void test_rk4_against_analytical_solution() {
    const atj::JointModel model(default_parameters());

    atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    for (int i = 0; i < steps; ++i) {
        state = atj::rk4_step(model, state, input, dt);
    }

    // Analytical position at t = 1.0 s for Model v0.
    constexpr double expected_q = 0.3343086493333882;

    expect_near(state.q, expected_q, 1e-10, "RK4 position at t = 1 second");
}

} // namespace

int main() {
    test_euler_first_step();
    test_equilibrium_remains_equilibrium();
    test_rk4_against_analytical_solution();

    std::cout << "All integrator tests passed.\n";

    return EXIT_SUCCESS;
}
