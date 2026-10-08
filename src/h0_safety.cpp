#include "atj/h0_safety.hpp"

#include <cmath>
#include <stdexcept>

namespace atj::h0 {

namespace {

int sign(double value) {
    if (value > 0.0) {
        return 1;
    }

    if (value < 0.0) {
        return -1;
    }

    return 0;
}

bool finite(double value) {
    return std::isfinite(value);
}

} // namespace

SafetySupervisor::SafetySupervisor(SafetyLimits limits) : limits_(limits) {
    if (limits_.max_abs_joint_angle_rad <= 0.0 || limits_.max_abs_motor_winding_m <= 0.0 ||
        limits_.max_command_step_m <= 0.0 || limits_.direction_movement_threshold_m <= 0.0 ||
        limits_.direction_check_timeout_s <= 0.0 || limits_.tracking_error_grace_s <= 0.0 ||
        limits_.max_tracking_error_m <= 0.0 || limits_.target_tolerance_m <= 0.0 ||
        limits_.max_motion_duration_s <= 0.0) {
        throw std::invalid_argument("Invalid H0 safety limits");
    }
}

SafetyState SafetySupervisor::state() const {
    return state_;
}

SafetyFault SafetySupervisor::fault() const {
    return fault_;
}

bool SafetySupervisor::driver_enable_allowed() const {
    return state_ == SafetyState::moving || state_ == SafetyState::holding;
}

void SafetySupervisor::latch_fault(SafetyFault fault) {
    fault_ = fault;
    state_ = SafetyState::fault;
}

void SafetySupervisor::clear_fault() {
    if (state_ == SafetyState::fault) {
        state_ = SafetyState::disabled;
        fault_ = SafetyFault::none;
    }
}

void SafetySupervisor::disable() {
    /*
     * Do not erase a latched fault merely because
     * someone requested driver disable.
     */
    if (state_ != SafetyState::fault) {
        state_ = SafetyState::disabled;
        fault_ = SafetyFault::none;
    }
}

bool SafetySupervisor::validate_sample(const SafetySample& sample) {
    if (!finite(sample.time_s) || !finite(sample.joint_angle_rad) ||
        !finite(sample.winding.motor_1_m) || !finite(sample.winding.motor_2_m)) {
        latch_fault(SafetyFault::non_finite_measurement);

        return false;
    }

    if (std::abs(sample.joint_angle_rad) > limits_.max_abs_joint_angle_rad) {
        latch_fault(SafetyFault::joint_angle_limit);

        return false;
    }

    if (std::abs(sample.winding.motor_1_m) > limits_.max_abs_motor_winding_m) {
        latch_fault(SafetyFault::motor_1_displacement_limit);

        return false;
    }

    if (std::abs(sample.winding.motor_2_m) > limits_.max_abs_motor_winding_m) {
        latch_fault(SafetyFault::motor_2_displacement_limit);

        return false;
    }

    return true;
}

bool SafetySupervisor::validate_target(const MotionTarget& target, const SafetySample& sample) {
    if (!finite(target.winding.motor_1_m) || !finite(target.winding.motor_2_m)) {
        latch_fault(SafetyFault::non_finite_target);

        return false;
    }

    if (std::abs(target.winding.motor_1_m) > limits_.max_abs_motor_winding_m) {
        latch_fault(SafetyFault::motor_1_displacement_limit);

        return false;
    }

    if (std::abs(target.winding.motor_2_m) > limits_.max_abs_motor_winding_m) {
        latch_fault(SafetyFault::motor_2_displacement_limit);

        return false;
    }

    const double step_1 = target.winding.motor_1_m - sample.winding.motor_1_m;
    const double step_2 = target.winding.motor_2_m - sample.winding.motor_2_m;

    if (std::abs(step_1) > limits_.max_command_step_m) {
        latch_fault(SafetyFault::motor_1_command_step_limit);

        return false;
    }

    if (std::abs(step_2) > limits_.max_command_step_m) {
        latch_fault(SafetyFault::motor_2_command_step_limit);

        return false;
    }

    return true;
}

bool SafetySupervisor::arm(const SafetySample& sample) {
    if (state_ == SafetyState::fault) {
        return false;
    }

    if (state_ != SafetyState::disabled) {
        latch_fault(SafetyFault::invalid_state);

        return false;
    }

    if (!validate_sample(sample)) {
        return false;
    }

    state_ = SafetyState::armed;
    fault_ = SafetyFault::none;

    last_time_s_ = sample.time_s;

    return true;
}

bool SafetySupervisor::begin_motion(const MotionTarget& target, const SafetySample& sample) {
    if (state_ == SafetyState::fault) {
        return false;
    }

    if (state_ != SafetyState::armed && state_ != SafetyState::holding) {
        latch_fault(SafetyFault::invalid_state);

        return false;
    }

    if (!validate_sample(sample)) {
        return false;
    }

    if (!validate_target(target, sample)) {
        return false;
    }

    target_ = target;
    motion_start_ = sample;

    last_time_s_ = sample.time_s;

    expected_direction_1_ = sign(target.winding.motor_1_m - sample.winding.motor_1_m);
    expected_direction_2_ = sign(target.winding.motor_2_m - sample.winding.motor_2_m);

    if (target_reached(sample)) {
        state_ = SafetyState::holding;
    } else {
        state_ = SafetyState::moving;
    }

    return true;
}

bool SafetySupervisor::target_reached(const SafetySample& sample) const {
    const double error_1 = target_.winding.motor_1_m - sample.winding.motor_1_m;
    const double error_2 = target_.winding.motor_2_m - sample.winding.motor_2_m;

    return std::abs(error_1) <= limits_.target_tolerance_m &&
           std::abs(error_2) <= limits_.target_tolerance_m;
}

void SafetySupervisor::update(const SafetySample& sample) {
    if (state_ == SafetyState::disabled || state_ == SafetyState::fault) {
        return;
    }

    if (!validate_sample(sample)) {
        return;
    }

    if (sample.time_s < last_time_s_) {
        latch_fault(SafetyFault::non_monotonic_time);

        return;
    }

    last_time_s_ = sample.time_s;

    /*
     * ARMED has no active motor target.
     * We still validate the physical measurement above.
     */
    if (state_ == SafetyState::armed) {
        return;
    }

    const double tracking_error_1 = target_.winding.motor_1_m - sample.winding.motor_1_m;
    const double tracking_error_2 = target_.winding.motor_2_m - sample.winding.motor_2_m;

    /*
     * HOLDING:
     *
     * Drivers stay enabled, but excessive drift from the position target faults immediately.
     */
    if (state_ == SafetyState::holding) {
        if (std::abs(tracking_error_1) > limits_.max_tracking_error_m) {
            latch_fault(SafetyFault::motor_1_tracking_error);

            return;
        }

        if (std::abs(tracking_error_2) > limits_.max_tracking_error_m) {
            latch_fault(SafetyFault::motor_2_tracking_error);

            return;
        }

        return;
    }

    const double elapsed = sample.time_s - motion_start_.time_s;

    const double observed_motion_1 = sample.winding.motor_1_m - motion_start_.winding.motor_1_m;

    const double observed_motion_2 = sample.winding.motor_2_m - motion_start_.winding.motor_2_m;

    /*
     * Direction-consistency checks.
     *
     * All of these operate in normalized winding coordinates:
     *
     *      positive -> WIND
     *      negative -> UNWIND
     *
     * Raw encoder polarity never appears here.
     */
    if (expected_direction_1_ > 0 && observed_motion_1 < -limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_1_direction_mismatch);

        return;
    }

    if (expected_direction_1_ < 0 && observed_motion_1 > limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_1_direction_mismatch);

        return;
    }

    if (expected_direction_2_ > 0 && observed_motion_2 > -limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_2_direction_mismatch);

        return;
    }

    if (expected_direction_2_ < 0 && observed_motion_2 > limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_2_direction_mismatch);

        return;
    }

    /*
     * If a motor was supposed to stay still, moving by more
     * than the direction threshold is also suspicous.
     */
    if (expected_direction_1_ == 0 &&
        std::abs(observed_motion_1) > limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_1_direction_mismatch);

        return;
    }

    if (expected_direction_2_ == 0 &&
        std::abs(observed_motion_2) > limits_.direction_movement_threshold_m) {
        latch_fault(SafetyFault::motor_2_direction_mismatch);

        return;
    }

    /*
     * Short direction/no-motion timeout.
     *
     * If motion was commanded but there is not even a small
     * amount of encoder motion promptly, something is wrong.
     */
    if (elapsed >= limits_.direction_check_timeout_s) {
        if (expected_direction_1_ != 0 &&
            std::abs(observed_motion_1) < limits_.direction_movement_threshold_m) {
            latch_fault(SafetyFault::motor_1_no_motion);

            return;
        }

        if (expected_direction_2_ != 0 &&
            std::abs(observed_motion_2) < limits_.direction_movement_threshold_m) {
            latch_fault(SafetyFault::motor_2_no_motion);

            return;
        }
    }

    /*
     * After a short grace period, large position-following
     * error is no longer acceptable.
     */
    if (elapsed >= limits_.tracking_error_grace_s) {
        if (std::abs(tracking_error_1) > limits_.max_tracking_error_m) {
            latch_fault(SafetyFault::motor_1_tracking_error);

            return;
        }

        if (std::abs(tracking_error_2) > limits_.max_tracking_error_m) {
            latch_fault(SafetyFault::motor_2_tracking_error);

            return;
        }
    }

    if (elapsed > limits_.max_motion_duration_s) {
        latch_fault(SafetyFault::motion_timeout);

        return;
    }

    if (target_reached(sample)) {
        state_ = SafetyState::holding;
    }
}

} // namespace atj::h0
