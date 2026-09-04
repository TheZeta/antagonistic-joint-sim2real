#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

constexpr double tolerance = 1e-9;

atj::Parameters default_parameters() {
    return atj::Parameters{.inertia = 0.01,
                           .damping = 0.02,
                           .spring_stiffness = 200.0,
                           .joint_radius = 0.02,
                           .motor_radius = 0.01,
                           .preload = 0.01};
}

} // namespace

TEST_CASE("Euler integration produces expected first step") {
    const atj::JointModel model(default_parameters());

    const atj::State initial_state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.01;

    const atj::State result = atj::euler_step(model, initial_state, input, dt);

    REQUIRE(result.q == Catch::Approx(0.0).margin(tolerance));

    REQUIRE(result.q_dot == Catch::Approx(0.04).margin(tolerance));
}

TEST_CASE("Numerical integrators preserve static equilibrium") {
    const atj::JointModel model(default_parameters());

    const atj::State equilibrium{.q = 0.25, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.01;

    const atj::State euler_result = atj::euler_step(model, equilibrium, input, dt);

    const atj::State rk4_result = atj::rk4_step(model, equilibrium, input, dt);

    SECTION("Euler") {
        REQUIRE(euler_result.q == Catch::Approx(0.25).margin(tolerance));

        REQUIRE(euler_result.q_dot == Catch::Approx(0.0).margin(tolerance));
    }

    SECTION("RK4") {
        REQUIRE(rk4_result.q == Catch::Approx(0.25).margin(tolerance));

        REQUIRE(rk4_result.q_dot == Catch::Approx(0.0).margin(tolerance));
    }
}

TEST_CASE("RK4 matches analytical solution at one second") {
    const atj::JointModel model(default_parameters());

    atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    for (int i = 0; i < steps; ++i) {
        state = atj::rk4_step(model, state, input, dt);
    }

    constexpr double analytical_q = 0.33430864933338816;

    REQUIRE(state.q == Catch::Approx(analytical_q).margin(1e-10));
}
