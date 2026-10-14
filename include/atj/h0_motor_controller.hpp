#pragma once

#include <cstdint>

namespace atj::h0 {

enum class MotorServoMode { coast, brake, wind, unwind };

struct MotorServoOutput {
    MotorServoMode mode = MotorServoMode::coast;

    std::uint32_t duty_percent = 0;

    double velocity_counts_per_s = 0.0;
    double desired_velocity_counts_per_s = 0.0;
};

struct DualMotorServoOutput {
    MotorServoOutput motor_1;
    MotorServoOutput motor_2;
};

struct MotorServoConfig {
    std::int32_t position_tolerance_counts = 1;
    double position_gain_per_s = 20.0;
    double max_velocity_counts_per_s = 80.0;
    double velocity_filter_time_constant_s = 0.020;
    std::uint32_t drive_duty_percent = 20;
};

[[nodiscard]]
constexpr MotorServoConfig h0_motor_servo_config() {
    return {};
}

class MotorPositionServo {
  public:
    explicit MotorPositionServo(MotorServoConfig config = h0_motor_servo_config());
    void reset();

    [[nodiscard]]
    MotorServoOutput update(double time_s, std::int32_t current_count, std::int32_t target_count,
                            bool enabled);

  private:
    MotorServoConfig config_;
    bool initialized_ = false;
    double previous_time_s_ = 0.0;
    std::int32_t previous_count_ = 0;
    double filtered_velocity_counts_per_s_ = 0.0;

    void update_velocity(double time_s, std::int32_t current_count);
};

} // namespace atj::h0
