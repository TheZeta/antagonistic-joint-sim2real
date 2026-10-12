#include "atj/h0_device.hpp"
#include "atj/h0_limits.hpp"

#include <catch2/catch_test_macros.hpp>

namespace {

atj::h0::SafetyLimits test_limits() {
    return atj::h0::SafetyLimits{.max_abs_joint_angle_rad = 1.0,
                                 .max_abs_motor_winding_m = 0.030,

                                 .max_command_step_m = 0.005,

                                 .direction_movement_threshold_m = 0.0002,
                                 .direction_check_timeout_s = 0.030,

                                 .tracking_error_grace_s = 0.050,
                                 .max_tracking_error_m = 0.001,

                                 .target_tolerance_m = 0.0002,

                                 .max_motion_duration_s = 0.250};
}

atj::h0::RawHardwareSample initial_sample() {
    return atj::h0::RawHardwareSample{.time_us = 1'000'000,
                                      .qraw = 3840,

                                      .motor_1_raw_count = 1000,
                                      .motor_2_raw_count = 2000};
}

} // namespace

TEST_CASE("H0 dry-run device starts with drivers disabled") {
    atj::h0::H0Device device(test_limits());

    device.ingest_sample(initial_sample());

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::disabled);

    REQUIRE_FALSE(telemetry.driver_enabled);
}

TEST_CASE("H0 ARM captures current motor encoder baselines") {
    atj::h0::H0Device device(test_limits());

    device.ingest_sample(initial_sample());

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::armed);

    REQUIRE(telemetry.motor_1_baseline_count == 1000);
    REQUIRE(telemetry.motor_2_baseline_count == 2000);

    REQUIRE(telemetry.motor_1_winding_counts == 0);
    REQUIRE(telemetry.motor_2_winding_counts == 0);

    REQUIRE_FALSE(telemetry.driver_enabled);
}

TEST_CASE("H0 dry-run telemetry reports normalized winding counts") {
    atj::h0::H0Device device(test_limits());

    device.ingest_sample(initial_sample());

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    auto moved = initial_sample();

    moved.time_us = 1'010'000;

    /*
     * Motor 1 WIND:
     * raw count decreases by 201.
     *
     * Motor 2 UNWIND:
     * raw count increases by 201.
     */
    moved.motor_1_raw_count = 799;
    moved.motor_2_raw_count = 2201;

    device.ingest_sample(moved);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.motor_1_winding_counts == 201);
    REQUIRE(telemetry.motor_2_winding_counts == -201);

    REQUIRE_FALSE(telemetry.driver_enabled);
}

TEST_CASE("H0 dry-run firmware rejects MOVE commands") {
    atj::h0::H0Device device(test_limits());

    device.ingest_sample(initial_sample());

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    const atj::h0::HostCommand move{.sequence = 2,
                                    .type = atj::h0::CommandType::move,

                                    .motor_1_target_winding_counts = 100,
                                    .motor_2_target_winding_counts = -100};

    REQUIRE(device.handle_command(move) == atj::h0::CommandDisposition::rejected_motion_inhibited);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::armed);

    REQUIRE_FALSE(telemetry.driver_enabled);

    REQUIRE(telemetry.motor_1_target_winding_counts == 0);
    REQUIRE(telemetry.motor_2_target_winding_counts == 0);
}

TEST_CASE("H0 DISABLE requires fresh baselines before next run") {
    atj::h0::H0Device device(test_limits());

    device.ingest_sample(initial_sample());

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    const atj::h0::HostCommand disable{.sequence = 2, .type = atj::h0::CommandType::disable};

    REQUIRE(device.handle_command(disable) == atj::h0::CommandDisposition::accepted);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::disabled);

    REQUIRE(telemetry.motor_1_winding_counts == 0);
    REQUIRE(telemetry.motor_2_winding_counts == 0);

    REQUIRE_FALSE(telemetry.driver_enabled);
}

TEST_CASE("H0 single-motor authority accepts bounded Motor 1 WIND move") {
    atj::h0::H0Device device(atj::h0::h0_safety_limits(), atj::h0::MotionAuthority::coordinated);

    const auto initial = initial_sample();

    device.ingest_sample(initial);

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    const atj::h0::HostCommand move{.sequence = 2,
                                    .type = atj::h0::CommandType::move,
                                    .motor_1_target_winding_counts = 5,
                                    .motor_2_target_winding_counts = 0};

    REQUIRE(device.handle_command(move) == atj::h0::CommandDisposition::accepted);

    const auto drive = device.drive_command();

    REQUIRE(drive.has_value());

    REQUIRE(drive->motor_1 == atj::h0::MotorDriveDirection::wind);
    REQUIRE(drive->motor_2 == atj::h0::MotorDriveDirection::off);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::moving);
    REQUIRE(telemetry.motor_1_target_winding_counts == 5);
}

TEST_CASE("H0 power authority accepts coordinated move") {
    atj::h0::H0Device device(atj::h0::h0_safety_limits(), atj::h0::MotionAuthority::coordinated);

    device.ingest_sample(initial_sample());

    const atj::h0::HostCommand arm{.sequence = 1, .type = atj::h0::CommandType::arm};

    REQUIRE(device.handle_command(arm) == atj::h0::CommandDisposition::accepted);

    const atj::h0::HostCommand move{.sequence = 2,
                                    .type = atj::h0::CommandType::move,
                                    .motor_1_target_winding_counts = 5,
                                    .motor_2_target_winding_counts = -5};

    REQUIRE(device.handle_command(move) == atj::h0::CommandDisposition::accepted);

    const auto drive = device.drive_command();

    REQUIRE(drive.has_value());

    REQUIRE(drive->motor_1 == atj::h0::MotorDriveDirection::wind);
    REQUIRE(drive->motor_2 == atj::h0::MotorDriveDirection::unwind);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::moving);
    REQUIRE(telemetry.motor_1_target_winding_counts == 5);
    REQUIRE(telemetry.motor_2_target_winding_counts == -5);
}

TEST_CASE("H0 correct encoder motion reaches holding") {
    atj::h0::H0Device device(atj::h0::h0_safety_limits(), atj::h0::MotionAuthority::coordinated);

    const auto initial = initial_sample();

    device.ingest_sample(initial);

    REQUIRE(device.handle_command({.sequence = 1, .type = atj::h0::CommandType::arm}) ==
            atj::h0::CommandDisposition::accepted);

    REQUIRE(device.handle_command({.sequence = 2,
                                   .type = atj::h0::CommandType::move,
                                   .motor_1_target_winding_counts = 5,
                                   .motor_2_target_winding_counts = 0}) ==
            atj::h0::CommandDisposition::accepted);

    auto moved = initial;

    moved.time_us += 10'000;

    /*
     * WIND -> raw count decreases.
     *
     * baseline 1000 -> raw 995
     * normalized winding = +5.
     */
    moved.motor_1_raw_count = 995;

    device.ingest_sample(moved);

    REQUIRE(device.make_telemetry().state == atj::h0::SafetyState::holding);

    const auto drive = device.drive_command();

    REQUIRE(drive.has_value());

    REQUIRE(drive->motor_1 == atj::h0::MotorDriveDirection::off);
}

TEST_CASE("H0 wrong encoder direction faults after one sample") {
    atj::h0::H0Device device(atj::h0::h0_safety_limits(), atj::h0::MotionAuthority::coordinated);

    const auto initial = initial_sample();

    device.ingest_sample(initial);

    REQUIRE(device.handle_command({.sequence = 1, .type = atj::h0::CommandType::arm}) ==
            atj::h0::CommandDisposition::accepted);

    REQUIRE(device.handle_command({.sequence = 2,
                                   .type = atj::h0::CommandType::move,
                                   .motor_1_target_winding_counts = 5,
                                   .motor_2_target_winding_counts = 0}) ==
            atj::h0::CommandDisposition::accepted);

    auto wrong = initial;

    wrong.time_us += 10'000;

    /*
     * Commanded WIND, but raw encoder increased:
     *
     * normalized displacement = -5 counts.
     */
    wrong.motor_1_raw_count = 1005;

    device.ingest_sample(wrong);

    const auto telemetry = device.make_telemetry();

    REQUIRE(telemetry.state == atj::h0::SafetyState::fault);
    REQUIRE(telemetry.fault == atj::h0::SafetyFault::motor_1_direction_mismatch);

    const auto drive = device.drive_command();

    REQUIRE(drive.has_value());

    REQUIRE(drive->motor_1 == atj::h0::MotorDriveDirection::off);
    REQUIRE(drive->motor_2 == atj::h0::MotorDriveDirection::off);
}
