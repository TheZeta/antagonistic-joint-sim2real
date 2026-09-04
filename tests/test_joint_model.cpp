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

TEST_CASE("Initial preload produces equal tendon forces") {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 0.0, .theta_2 = 0.0};

    REQUIRE(model.tendon_1_extension(state, input) == Catch::Approx(0.01).margin(tolerance));

    REQUIRE(model.tendon_2_extension(state, input) == Catch::Approx(0.01).margin(tolerance));

    REQUIRE(model.tendon_1_force(state, input) == Catch::Approx(2.0).margin(tolerance));

    REQUIRE(model.tendon_2_force(state, input) == Catch::Approx(2.0).margin(tolerance));

    const auto derivative = model.derivative(state, input);

    REQUIRE(derivative.q_ddot == Catch::Approx(0.0).margin(tolerance));
}

TEST_CASE("Differential actuation accelerates the joint") {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    REQUIRE(model.tendon_1_extension(state, input) == Catch::Approx(0.02).margin(tolerance));

    REQUIRE(model.tendon_2_extension(state, input) == Catch::Approx(0.01).margin(tolerance));

    REQUIRE(model.tendon_1_force(state, input) == Catch::Approx(4.0).margin(tolerance));

    REQUIRE(model.tendon_2_force(state, input) == Catch::Approx(2.0).margin(tolerance));

    const auto derivative = model.derivative(state, input);

    REQUIRE(derivative.q_ddot == Catch::Approx(4.0).margin(tolerance));
}

TEST_CASE("Predicted static equilibrium has zero acceleration") {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.25, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    REQUIRE(model.tendon_1_force(state, input) == Catch::Approx(3.0).margin(tolerance));

    REQUIRE(model.tendon_2_force(state, input) == Catch::Approx(3.0).margin(tolerance));

    const auto derivative = model.derivative(state, input);

    REQUIRE(derivative.q_ddot == Catch::Approx(0.0).margin(tolerance));
}

TEST_CASE("Common-mode actuation increases tension without acceleration") {
    const atj::JointModel model(default_parameters());

    const atj::State state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 1.0};

    REQUIRE(model.tendon_1_extension(state, input) == Catch::Approx(0.02).margin(tolerance));

    REQUIRE(model.tendon_2_extension(state, input) == Catch::Approx(0.02).margin(tolerance));

    REQUIRE(model.tendon_1_force(state, input) == Catch::Approx(4.0).margin(tolerance));

    REQUIRE(model.tendon_2_force(state, input) == Catch::Approx(4.0).margin(tolerance));

    const auto derivative = model.derivative(state, input);

    REQUIRE(derivative.q_ddot == Catch::Approx(0.0).margin(tolerance));
}
