#pragma once

#include "atj/h0_safety.hpp"

#include <numbers>

namespace atj::h0 {

inline constexpr double h0_max_abs_joint_angle_rad = 30.0 * std::numbers::pi / 180.0;
inline constexpr double h0_max_abs_motor_winding_m = 0.016;

/*
 * Initial powered-motion limits.
 *
 * These are intentionally conservative and may later be
 * tightened from experimental evidence.
 */
inline constexpr double h0_max_command_step_m = 0.002;
inline constexpr double h0_direction_movement_threshold_m = 0.0003;
inline constexpr double h0_direction_check_timeout_s = 0.150;
inline constexpr double h0_tracking_error_grace_s = 0.250;
inline constexpr double h0_max_tracking_error_m = 0.0015;
inline constexpr double h0_target_tolerance_m = 0.0003;
inline constexpr double h0_max_motion_duration_s = 2.0;

[[nodiscard]]
constexpr SafetyLimits h0_safety_limits() {
    return SafetyLimits{.max_abs_joint_angle_rad = h0_max_abs_joint_angle_rad,
                        .max_abs_motor_winding_m = h0_max_abs_motor_winding_m,
                        .max_command_step_m = h0_max_command_step_m,
                        .direction_movement_threshold_m = h0_direction_movement_threshold_m,
                        .direction_check_timeout_s = h0_direction_check_timeout_s,
                        .tracking_error_grace_s = h0_tracking_error_grace_s,
                        .max_tracking_error_m = h0_max_tracking_error_m,
                        .target_tolerance_m = h0_target_tolerance_m,
                        .max_motion_duration_s = h0_max_motion_duration_s};
}

} // namespace atj::h0
