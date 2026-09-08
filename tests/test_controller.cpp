#include "atj/controller.hpp"
#include "atj/joint_model.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <numbers>

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

TEST_CASE("P controller produces differential command from position error") {
    const atj::PositionPController controller(8.0);

    SECTION("Zero error produces zero command") {
        const atj::State state{.q = 0.25, .q_dot = 0.0};

        REQUIRE(controller.differential_command(0.25, state) ==
                Catch::Approx(0.0).margin(tolerance));
    }

    SECTION("Positive error produces positive command") {
        const atj::State state{.q = 0.10, .q_dot = 0.0};

        REQUIRE(controller.differential_command(0.20, state) ==
                Catch::Approx(0.8).margin(tolerance));
    }

    SECTION("Negative error produces negative command") {
        const atj::State state{.q = 0.30, .q_dot = 0.0};

        REQUIRE(controller.differential_command(0.20, state) ==
                Catch::Approx(-0.8).margin(tolerance));
    }
}

TEST_CASE("Antagonistic input mapping preserves differential and common mode") {
    constexpr double theta_d = 1.4;
    constexpr double theta_c = 0.8;

    const atj::Input input = atj::make_antagonistic_input(theta_d, theta_c);

    REQUIRE(input.theta_1 - input.theta_2 == Catch::Approx(theta_d).margin(tolerance));

    REQUIRE((input.theta_1 + input.theta_2) / 2.0 == Catch::Approx(theta_c).margin(tolerance));
}

TEST_CASE("P-controlled predicted equilibrium has zero plant acceleration") {
    const atj::Parameters parameters = default_parameters();

    const atj::JointModel model(parameters);

    constexpr double kp = 8.0;

    const atj::PositionPController controller(kp);

    constexpr double q_reference = 20.0 * std::numbers::pi / 180.0;

    const double rotational_stiffness =
        2.0 * parameters.spring_stiffness * parameters.joint_radius * parameters.joint_radius;

    const double input_gain =
        parameters.spring_stiffness * parameters.joint_radius * parameters.motor_radius;

    const double q_equilibrium =
        (input_gain * kp / (rotational_stiffness + input_gain * kp)) * q_reference;

    const atj::State state{.q = q_equilibrium, .q_dot = 0.0};

    const double theta_d = controller.differential_command(q_reference, state);

    const atj::Input input = atj::make_antagonistic_input(theta_d, 1.0);

    const auto derivative = model.derivative(state, input);

    REQUIRE(derivative.q_ddot == Catch::Approx(0.0).margin(tolerance));
}
