#include "atj/h0_calibration.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <numbers>
#include <stdexcept>

namespace {

constexpr double tolerance = 1e-12;

} // namespace

TEST_CASE("H0 AS5600 geometric zero maps to q equals zero") {
    REQUIRE(atj::h0::joint_angle_rad_from_raw(atj::h0::joint_zero_raw) ==
            Catch::Approx(0.0).margin(tolerance));
}

TEST_CASE("H0 AS5600 sign matches commissioned joint sign") {
    constexpr double radians_per_count = 2.0 * std::numbers::pi / 4096.0;

    SECTION("Increasing QRAW produces positive q") {
        REQUIRE(atj::h0::joint_angle_rad_from_raw(atj::h0::joint_zero_raw + 1) ==
                Catch::Approx(radians_per_count).margin(tolerance));
    }

    SECTION("Decreasing QRAW produces negative q") {
        REQUIRE(atj::h0::joint_angle_rad_from_raw(atj::h0::joint_zero_raw - 1) ==
                Catch::Approx(-radians_per_count).margin(tolerance));
    }
}

TEST_CASE("H0 AS5600 mapping is continuous across raw rollover") {
    const double q_4095 = atj::h0::joint_angle_rad_from_raw(4095);

    const double q_0 = atj::h0::joint_angle_rad_from_raw(0);

    constexpr double radians_per_count = 2.0 * std::numbers::pi / 4096.0;

    /*
     * Raw 4095 -> raw 0 is one positive encoder count.
     */
    REQUIRE(q_0 - q_4095 == Catch::Approx(radians_per_count).margin(tolerance));
}

TEST_CASE("H0 AS5600 half revolution uses negative pi branch") {
    const int half_turn_raw = (atj::h0::joint_zero_raw + atj::h0::as5600_half_revolution_counts) %
                              atj::h0::as5600_counts_per_revolution;

    REQUIRE(atj::h0::joint_angle_rad_from_raw(half_turn_raw) ==
            Catch::Approx(-std::numbers::pi).margin(tolerance));
}

TEST_CASE("H0 rejects invalid AS5600 raw values") {
    REQUIRE_THROWS_AS(atj::h0::joint_angle_rad_from_raw(-1), std::out_of_range);

    REQUIRE_THROWS_AS(atj::h0::joint_angle_rad_from_raw(4096), std::out_of_range);
}

TEST_CASE("H0 motor encoder calibration preserves WIND sign") {
    SECTION("WIND produces positive winding displacement") {
        REQUIRE(atj::h0::winding_displacement_m_from_count_delta(-201) ==
                Catch::Approx(0.023).margin(tolerance));
    }

    SECTION("UNWIND produces negative winding displacement") {
        REQUIRE(atj::h0::winding_displacement_m_from_count_delta(201) ==
                Catch::Approx(-0.023).margin(tolerance));
    }
}

TEST_CASE("H0 tendon calibration round-trips displacement") {
    constexpr double displacement = 0.017;

    const double count_delta = atj::h0::motor_count_delta_from_winding_displacement_m(displacement);

    const double reconstructed_displacement = -count_delta * atj::h0::meters_per_motor_count;

    REQUIRE(reconstructed_displacement == Catch::Approx(displacement).margin(tolerance));
}

TEST_CASE("H0 positive joint motion crosses AS5600 rollover correctly") {
    constexpr double expected_angle = 22.5 * std::numbers::pi / 180.0;

    REQUIRE(atj::h0::joint_angle_rad_from_raw(0) ==
            Catch::Approx(expected_angle).margin(tolerance));
}
