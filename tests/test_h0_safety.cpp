#include "atj/h0_safety.hpp"

#include <catch2/catch_test_macros.hpp>

namespace {

atj::h0::SafetyLimits test_limits() {
    return atj::h0::SafetyLimits{.max_abs_joint_angle_rad = 0.50,
                                 .max_abs_motor_winding_m = 0.030,

                                 .max_command_step_m = 0.005,

                                 .direction_movement_threshold_m = 0.0002,
                                 .direction_check_timeout_s = 0.030,

                                 .tracking_error_grace_s = 0.050,
                                 .max_tracking_error_m = 0.001,

                                 .target_tolerance_m = 0.0002,

                                 .max_motion_duration_s = 0.250};
}

atj::h0::SafetySample safe_sample(double time_s = 0.0) {
    return atj::h0::SafetySample{
        .time_s = time_s, .joint_angle_rad = 0.0, .winding = {.motor_1_m = 0.0, .motor_2_m = 0.0}};
}

} // namespace

TEST_CASE("H0 safety supervisor starts fail-safe disabled") {
    const atj::h0::SafetySupervisor supervisor(test_limits());

    REQUIRE(supervisor.state() == atj::h0::SafetyState::disabled);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 arming does not enable motor drivers") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    REQUIRE(supervisor.arm(safe_sample()));
    REQUIRE(supervisor.state() == atj::h0::SafetyState::armed);
    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 valid bounded motion enables drivers") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto sample = safe_sample();

    REQUIRE(supervisor.arm(sample));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = -0.003}};

    REQUIRE(supervisor.begin_motion(target, sample));
    REQUIRE(supervisor.state() == atj::h0::SafetyState::moving);
    REQUIRE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 wrong encoder motion direction latches fault") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = 0.0}};

    REQUIRE(supervisor.begin_motion(target, initial));

    auto wrong_direction = safe_sample(0.010);

    /*
     * Motor 1 was commanded toward positive winding (WIND)
     * but physically moved negative (UNWIND).
     */
    wrong_direction.winding.motor_1_m = -0.0005;

    supervisor.update(wrong_direction);

    REQUIRE(supervisor.state() == atj::h0::SafetyState::fault);
    REQUIRE(supervisor.fault() == atj::h0::SafetyFault::motor_1_direction_mismatch);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 commanded motor motion must appear promptly") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = 0.0}};

    REQUIRE(supervisor.begin_motion(target, initial));

    supervisor.update(safe_sample(0.040));

    REQUIRE(supervisor.state() == atj::h0::SafetyState::fault);
    REQUIRE(supervisor.fault() == atj::h0::SafetyFault::motor_1_no_motion);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 joint-angle excursion latches fault") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = -0.003}};

    REQUIRE(supervisor.begin_motion(target, initial));

    auto sample = safe_sample(0.010);

    sample.joint_angle_rad = 0.60;

    supervisor.update(sample);

    REQUIRE(supervisor.fault() == atj::h0::SafetyFault::joint_angle_limit);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 rejects excessive motor command step") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.006, .motor_2_m = 0.0}};

    REQUIRE_FALSE(supervisor.begin_motion(target, initial));

    REQUIRE(supervisor.state() == atj::h0::SafetyState::fault);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 excessive position tracking error faults") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = 0.0}};

    REQUIRE(supervisor.begin_motion(target, initial));

    /*
     * Motor moved in the correct direction,
     * so direction consistency passes,
     * but after the tracking grace period it is
     * still much too far from its target.
     */
    auto sample = safe_sample(0.060);

    sample.winding.motor_1_m = 0.0005;

    supervisor.update(sample);

    REQUIRE(supervisor.fault() == atj::h0::SafetyFault::motor_1_tracking_error);

    REQUIRE_FALSE(supervisor.driver_enable_allowed());
}

TEST_CASE("H0 reached motor target enters holding") {
    atj::h0::SafetySupervisor supervisor(test_limits());

    const auto initial = safe_sample();

    REQUIRE(supervisor.arm(initial));

    const atj::h0::MotionTarget target{.winding = {.motor_1_m = 0.003, .motor_2_m = -0.003}};

    REQUIRE(supervisor.begin_motion(target, initial));

    auto reached = safe_sample(0.040);

    reached.winding.motor_1_m = 0.003;
    reached.winding.motor_2_m = -0.003;

    supervisor.update(reached);

    REQUIRE(supervisor.state() == atj::h0::SafetyState::holding);

    /*
     * Holding under tendon/spring load requires
     * the low-level position servos to remain active.
     */
    REQUIRE(supervisor.driver_enable_allowed());
}
