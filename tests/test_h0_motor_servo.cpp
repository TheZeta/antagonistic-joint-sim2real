#include "atj/h0_calibration.hpp"
#include "atj/h0_limits.hpp"
#include "atj/h0_motor_servo.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 servo winds toward positive winding target") {
    const auto command = atj::h0::single_motor_drive_command(0, 0, 10, 0);

    REQUIRE(command.has_value());
    REQUIRE(command->motor_1 == atj::h0::MotorDriveDirection::wind);
    REQUIRE(command->motor_2 == atj::h0::MotorDriveDirection::off);
}

TEST_CASE("H0 servo unwinds toward negative winding target") {
    const auto command = atj::h0::single_motor_drive_command(0, 0, -10, 0);

    REQUIRE(command.has_value());
    REQUIRE(command->motor_1 == atj::h0::MotorDriveDirection::unwind);
    REQUIRE(command->motor_2 == atj::h0::MotorDriveDirection::off);
}

TEST_CASE("H0 servo stops within encoder tolerance") {
    const auto command = atj::h0::single_motor_drive_command(8, 0, 10, 0);

    REQUIRE(command.has_value());
    REQUIRE(command->motor_1 == atj::h0::MotorDriveDirection::off);
}

TEST_CASE("H0 initial powered phase rejects coordinated motor motion") {
    const auto command = atj::h0::single_motor_drive_command(0, 0, 10, -10);

    REQUIRE_FALSE(command.has_value());
}

TEST_CASE("H0 servo tolerance lies inside safety target tolerance") {
    constexpr double servo_tolerance_m =
        static_cast<double>(atj::h0::h0_servo_tolerance_counts) * atj::h0::meters_per_motor_count;

    REQUIRE(servo_tolerance_m < atj::h0::h0_target_tolerance_m);
}
