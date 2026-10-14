#include "atj/h0_motor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace atj::h0 {

MotorPositionServo::MotorPositionServo(MotorServoConfig config) : config_(config) {}

void MotorPositionServo::reset() {
    initialized_ = false;
    previous_time_s_ = 0.0;
    previous_count_ = 0;
    filtered_velocity_counts_per_s_ = 0.0;
}

void MotorPositionServo::update_velocity(double time_s, std::int32_t current_count) {
    if (!initialized_) {
        initialized_ = true;
        previous_time_s_ = time_s;
        previous_count_ = current_count;
        filtered_velocity_counts_per_s_ = 0.0;

        return;
    }

    const double dt = time_s - previous_time_s_;

    if (dt <= 0.0) {
        return;
    }

    const auto count_delta =
        static_cast<std::int64_t>(current_count) - static_cast<std::int64_t>(previous_count_);
    const double raw_velocity = static_cast<double>(count_delta) / dt;

    /*
     * First-order low-pass filter.
     *
     * Encoder quantization at a ~1 kHz control rate would
     * otherwise produce samples such as:
     *
     *      0, 0, 1000, 0, 0 ...
     *
     * despite smooth physical rotation.
     */
    const double alpha = dt / (config_.velocity_filter_time_constant_s + dt);
    filtered_velocity_counts_per_s_ += alpha * (raw_velocity - filtered_velocity_counts_per_s_);

    previous_time_s_ = time_s;
    previous_count_ = current_count;
}

MotorServoOutput MotorPositionServo::update(double time_s, std::int32_t current_count,
                                            std::int32_t target_count, bool enabled) {
    if (!enabled) {
        reset();

        return {.mode = MotorServoMode::coast,
                .duty_percent = 0,
                .velocity_counts_per_s = 0.0,
                .desired_velocity_counts_per_s = 0.0};
    }

    update_velocity(time_s, current_count);

    const auto error =
        static_cast<std::int64_t>(target_count) - static_cast<std::int64_t>(current_count);

    /*
     * At the target we BRAKE rather than COAST.
     */
    if (std::abs(error) <= config_.position_tolerance_counts) {
        return {.mode = MotorServoMode::brake,
                .duty_percent = 0,
                .velocity_counts_per_s = filtered_velocity_counts_per_s_,
                .desired_velocity_counts_per_s = 0.0};
    }

    double desired_velocity = config_.position_gain_per_s * static_cast<double>(error);
    desired_velocity = std::clamp(desired_velocity, -config_.max_velocity_counts_per_s,
                                  config_.max_velocity_counts_per_s);

    /*
     * Positive normalized count motion = WIND.
     *
     * Negative normalized count motion = UNWIND.
     */
    if (error > 0) {
        /*
         * Already moving toward the target at least as
         * quickly as we want:
         *
         * stop accelerating and electrically brake.
         */
        if (filtered_velocity_counts_per_s_ >= desired_velocity) {
            return {.mode = MotorServoMode::brake,
                    .duty_percent = 0,
                    .velocity_counts_per_s = filtered_velocity_counts_per_s_,
                    .desired_velocity_counts_per_s = desired_velocity};
        }

        return {.mode = MotorServoMode::wind,
                .duty_percent = config_.drive_duty_percent,
                .velocity_counts_per_s = filtered_velocity_counts_per_s_,
                .desired_velocity_counts_per_s = desired_velocity};
    }

    /*
     * Negative error.
     *
     * When actual negative velocity is already more
     * negative than desired, brake instead of continuing
     * to accelerate toward / through the target.
     */
    if (filtered_velocity_counts_per_s_ <= desired_velocity) {
        return {.mode = MotorServoMode::brake,
                .duty_percent = 0,
                .velocity_counts_per_s = filtered_velocity_counts_per_s_,
                .desired_velocity_counts_per_s = desired_velocity};
    }

    return {.mode = MotorServoMode::unwind,
            .duty_percent = config_.drive_duty_percent,
            .velocity_counts_per_s = filtered_velocity_counts_per_s_,
            .desired_velocity_counts_per_s = desired_velocity};
}

} // namespace atj::h0
