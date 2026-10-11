#include "atj/h0_calibration.hpp"
#include "atj/h0_limits.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <numbers>

TEST_CASE("H0 motion authority uses conservative joint envelope") {
    REQUIRE(atj::h0::h0_max_abs_joint_angle_rad == Catch::Approx(std::numbers::pi / 6.0));
}

TEST_CASE("H0 motor displacement envelope is sixteen millimetres") {
    REQUIRE(atj::h0::h0_max_abs_motor_winding_m == Catch::Approx(0.016));
}

TEST_CASE("H0 motor envelope corresponds to approximately 140 encoder counts") {
    const double counts =
        atj::h0::motor_count_delta_from_winding_displacement_m(atj::h0::h0_max_abs_motor_winding_m);

    REQUIRE(counts == Catch::Approx(-139.826).margin(0.001));
}

TEST_CASE("H0 safety limit factory reproduces frozen constants") {
    constexpr auto limits = atj::h0::h0_safety_limits();

    REQUIRE(limits.max_abs_joint_angle_rad == atj::h0::h0_max_abs_joint_angle_rad);
    REQUIRE(limits.max_abs_motor_winding_m == atj::h0::h0_max_abs_motor_winding_m);
    REQUIRE(limits.max_command_step_m == atj::h0::h0_max_command_step_m);
}
