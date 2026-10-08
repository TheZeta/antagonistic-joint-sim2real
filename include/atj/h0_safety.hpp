#pragma once

#include "atj/h0_coordinates.hpp"

namespace atj::h0 {

enum class SafetyState { disabled = 0, armed = 1, moving = 2, holding = 3, fault = 4 };

enum class SafetyFault {
    none = 0,

    invalid_state = 1,
    non_finite_measurement = 2,
    non_finite_target = 3,
    non_monotonic_time = 4,

    joint_angle_limit = 5,

    motor_1_displacement_limit = 6,
    motor_2_displacement_limit = 7,

    motor_1_command_step_limit = 8,
    motor_2_command_step_limit = 9,

    motor_1_direction_mismatch = 10,
    motor_2_direction_mismatch = 11,

    motor_1_no_motion = 12,
    motor_2_no_motion = 13,

    motor_1_tracking_error = 14,
    motor_2_tracking_error = 15,

    motion_timeout = 16
};

struct SafetyLimits {
    double max_abs_joint_angle_rad;
    double max_abs_motor_winding_m;

    double max_command_step_m;

    double direction_movement_threshold_m;
    double direction_check_timeout_s;

    double tracking_error_grace_s;
    double max_tracking_error_m;

    double target_tolerance_m;

    double max_motion_duration_s;
};

struct SafetySample {
    double time_s;
    double joint_angle_rad;
    MotorWinding winding;
};

struct MotionTarget {
    MotorWinding winding;
};

class SafetySupervisor {
  public:
    explicit SafetySupervisor(SafetyLimits limits);

    [[nodiscard]]
    SafetyState state() const;

    [[nodiscard]]
    SafetyFault fault() const;

    [[nodiscard]]
    bool driver_enable_allowed() const;

    /*
     * Explicitly clear a latched fault.
     *
     * This never arms or enables the system.
     */
    void clear_fault();

    /*
     * Normal driver disable.
     *
     * A latched fault remains latched.
     */
    void disable();

    /*
     * Check that the current mechanism state is safe,
     * then enter ARMED.
     *
     * Drivers remain disabled while armed.
     */
    [[nodiscard]]
    bool arm(const SafetySample& sample);

    /*
     * Validate a bounded motor-position target ang begin motion.
     *
     * Allowed only from ARMED or HOLDING.
     */
    [[nodiscard]]
    bool begin_motion(const MotionTarget& target, const SafetySample& sample);

    /*
     * Feed every fresh hardware sample through the supervisor
     */
    void update(const SafetySample& sample);

  private:
    SafetyLimits limits_;

    SafetyState state_ = SafetyState::disabled;
    SafetyFault fault_ = SafetyFault::none;

    MotionTarget target_{};

    SafetySample motion_start_{};
    double last_time_s_ = 0.0;

    int expected_direction_1_ = 0;
    int expected_direction_2_ = 0;

    void latch_fault(SafetyFault fault);

    [[nodiscard]]
    bool validate_sample(const SafetySample& sample);

    [[nodiscard]]
    bool validate_target(const MotionTarget& target, const SafetySample& sample);

    [[nodiscard]]
    bool target_reached(const SafetySample& sample) const;
};

} // namespace atj::h0
