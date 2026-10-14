#include "atj/h0_motor_controller.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 motor servo coasts when disabled") {
    atj::h0::MotorPositionServo servo;

    const auto output = servo.update(0.0, 0, 10, false);

    REQUIRE(output.mode == atj::h0::MotorServoMode::coast);
    REQUIRE(output.duty_percent == 0);
}

TEST_CASE("H0 motor servo brakes at target") {
    atj::h0::MotorPositionServo servo;

    const auto output = servo.update(0.0, 10, 10, true);

    REQUIRE(output.mode == atj::h0::MotorServoMode::brake);
    REQUIRE(output.duty_percent == 0);
}

TEST_CASE("H0 motor servo winds toward positive error") {
    atj::h0::MotorPositionServo servo;

    const auto output = servo.update(0.0, 0, 5, true);

    REQUIRE(output.mode == atj::h0::MotorServoMode::wind);
    REQUIRE(output.duty_percent == 20);
}

TEST_CASE("H0 motor servo unwinds toward negative error") {
    atj::h0::MotorPositionServo servo;

    const auto output = servo.update(0.0, 0, -5, true);

    REQUIRE(output.mode == atj::h0::MotorServoMode::unwind);
    REQUIRE(output.duty_percent == 20);
}

TEST_CASE("H0 motor servo brakes before positive overshoot") {
    atj::h0::MotorPositionServo servo;

    /*
     * Establish initial estimator state.
     */
    servo.update(0.000, 0, 10, true);

    /*
     * Twenty milliseconds later the spool has advanced
     * four counts.
     *
     * Raw velocity = 200 counts/s.
     *
     * With 20 ms low-pass time constant, filtered
     * velocity becomes 100 counts/s.
     *
     * Remaining error = 6 counts.
     * Desired velocity is capped at 80 counts/s.
     *
     * Therefore the motor must BRAKE before reaching the
     * target.
     */
    const auto output = servo.update(0.020, 4, 10, true);

    REQUIRE(output.mode == atj::h0::MotorServoMode::brake);
    REQUIRE(output.velocity_counts_per_s > output.desired_velocity_counts_per_s);
}

TEST_CASE("H0 motor servo brakes before negative overshoot") {
    atj::h0::MotorPositionServo servo;

    servo.update(0.000, 0, -10, true);

    const auto output = servo.update(0.020, -4, -10, true);

    REQUIRE(output.mode == atj::h0::MotorServoMode::brake);
    REQUIRE(output.velocity_counts_per_s < output.desired_velocity_counts_per_s);
}

TEST_CASE("H0 motor servo corrects displacement from held target") {
    atj::h0::MotorPositionServo servo;

    const auto held = servo.update(0.000, 10, 10, true);

    REQUIRE(held.mode == atj::h0::MotorServoMode::brake);

    /*
     * Spring backdrives the spool two counts away
     */
    const auto disturbed = servo.update(0.020, 8, 10, true);

    REQUIRE(disturbed.mode == atj::h0::MotorServoMode::wind);
}
