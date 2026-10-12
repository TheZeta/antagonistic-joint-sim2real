#pragma once

#include <cstdint>

namespace atj::h0 {

enum class MotorDriveDirection { off, wind, unwind };

struct MotorDriveCommand {
    MotorDriveDirection motor_1;
    MotorDriveDirection motor_2;
};

/*
 * Two encoder counts correspond to about 0.229 mm.
 *
 * This lies safely inside the frozen 0.300 mm target
 * tolerance.
 */
inline constexpr std::int32_t h0_servo_tolerance_counts = 2;

[[nodiscard]]
constexpr MotorDriveDirection drive_direction(std::int32_t current_count,
                                              std::int32_t target_count) {
    const std::int32_t error = target_count - current_count;

    if (error > h0_servo_tolerance_counts) {
        return MotorDriveDirection::wind;
    }

    if (error < -h0_servo_tolerance_counts) {
        return MotorDriveDirection::unwind;
    }

    return MotorDriveDirection::off;
}

[[nodiscard]]
constexpr MotorDriveCommand
motor_drive_command(std::int32_t motor_1_current, std::int32_t motor_2_current,
                    std::int32_t motor_1_target, std::int32_t motor_2_target) {
    return MotorDriveCommand{.motor_1 = drive_direction(motor_1_current, motor_1_target),
                             .motor_2 = drive_direction(motor_2_current, motor_2_target)};
}
} // namespace atj::h0
