#pragma once

#include <cstdint>
#include <optional>

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

/*
 * Phase-1 powered motion permits at most ONE motor to
 * require motion at a time.
 *
 * nullopt therefore means:
 *
 *      coordinated motion requested -> reject
 */
[[nodiscard]]
constexpr std::optional<MotorDriveCommand>
single_motor_drive_command(std::int32_t motor_1_current, std::int32_t motor_2_current,
                           std::int32_t motor_1_target, std::int32_t motor_2_target) {
    const auto motor_1 = drive_direction(motor_1_current, motor_1_target);
    const auto motor_2 = drive_direction(motor_2_current, motor_2_target);

    if (motor_1 != MotorDriveDirection::off && motor_2 != MotorDriveDirection::off) {
        return std::nullopt;
    }

    return MotorDriveCommand{.motor_1 = motor_1, .motor_2 = motor_2};
}

} // namespace atj::h0
