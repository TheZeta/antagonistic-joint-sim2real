#include "atj/h0_coordinates.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <stdexcept>

namespace {

constexpr double tolerance = 1e-12;

} // namespace

TEST_CASE("H0 common WIND maps to positive common mode") {
    const atj::h0::MotorCountDelta counts{.motor_1 = -201, .motor_2 = -201};

    const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
    const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

    REQUIRE(coordinates.common_m == Catch::Approx(0.023).margin(tolerance));
    REQUIRE(coordinates.differential_m == Catch::Approx(0.0).margin(tolerance));
}

TEST_CASE("H0 common UNWIND maps to negative common mode") {
    const atj::h0::MotorCountDelta counts{.motor_1 = 201, .motor_2 = 201};

    const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
    const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

    REQUIRE(coordinates.common_m < 0.0);
    REQUIRE(coordinates.differential_m == Catch::Approx(0.0).margin(tolerance));
}

TEST_CASE("H0 commissioned q-positive actuation maps positiive") {
    /*
     * Commissioned q+ map:
     *
     *      (Delta C1, Delta C2) = (+N, -N)
     */
    const atj::h0::MotorCountDelta counts{
        .motor_1 = 201,
        .motor_2 = -201,
    };

    const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
    const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

    REQUIRE(coordinates.common_m == Catch::Approx(0.0).margin(tolerance));
    REQUIRE(coordinates.differential_m > 0.0);
}

TEST_CASE("H0 commissioned q-negative actuation maps negative") {
    /*
     * Commissioned q- map:
     *
     *      (Delta C1, Delta C2) = (-N, +N)
     */
    const atj::h0::MotorCountDelta counts{
        .motor_1 = -201,
        .motor_2 = 201,
    };

    const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
    const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

    REQUIRE(coordinates.common_m == Catch::Approx(0.0).margin(tolerance));
    REQUIRE(coordinates.differential_m < 0.0);
}

TEST_CASE("H0 individual motor winding signs match commissioning") {
    SECTION("Motor 1 WIND produces q-negative differential") {
        const atj::h0::MotorCountDelta counts{.motor_1 = -201, .motor_2 = 0};

        const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
        const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

        REQUIRE(coordinates.differential_m < 0.0);
    }

    SECTION("Motor 2 WIND produces q-positive differential") {
        const atj::h0::MotorCountDelta counts{.motor_1 = 0, .motor_2 = -201};

        const auto winding = atj::h0::motor_winding_from_count_deltas(counts);
        const auto coordinates = atj::h0::winding_coordinates_from_motor_winding(winding);

        REQUIRE(coordinates.differential_m > 0.0);
    }
}

TEST_CASE("H0 common differential transform round-trips") {
    const atj::h0::WindingCoordinates coordinates{.common_m = 0.012, .differential_m = 0.008};

    const auto winding = atj::h0::motor_winding_from_coordinates(coordinates);
    const auto reconstructed = atj::h0::winding_coordinates_from_motor_winding(winding);

    REQUIRE(reconstructed.common_m == Catch::Approx(coordinates.common_m).margin(tolerance));
    REQUIRE(reconstructed.differential_m ==
            Catch::Approx(coordinates.differential_m).margin(tolerance));
}

TEST_CASE("H0 physical motors map correctly into Model v0 tendons") {
    /*
     * Motor 2 is the positive-q tendon.
     * Motor 1 is the negative-q tendon.
     */
    const atj::h0::MotorWinding winding{.motor_1_m = -0.010, .motor_2_m = 0.010};

    constexpr double motor_radius_m = 0.010;

    const atj::Input input = atj::h0::model_input_from_motor_winding(winding, motor_radius_m);

    REQUIRE(input.theta_1 == Catch::Approx(1.0).margin(tolerance));
    REQUIRE(input.theta_2 == Catch::Approx(-1.0).margin(tolerance));
    REQUIRE(input.theta_1 - input.theta_2 > 0.0);
}

TEST_CASE("H0 model bridge rejects invalid motor radius") {
    const atj::h0::MotorWinding winding{.motor_1_m = 0.0, .motor_2_m = 0.0};

    REQUIRE_THROWS_AS(atj::h0::model_input_from_motor_winding(winding, 0.0), std::invalid_argument);
}
