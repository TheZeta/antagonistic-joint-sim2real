#include "atj/h0_device.hpp"

#include "atj/h0_calibration.hpp"

namespace atj::h0 {

H0Device::H0Device(SafetyLimits limits, MotionAuthority motion_authority)
    : safety_(limits), motion_authority_(motion_authority) {}

std::int32_t H0Device::motor_1_winding_counts() const {
    if (!baselines_valid_) {
        return 0;
    }

    return winding_count_from_raw(raw_.motor_1_raw_count, motor_1_baseline_count_);
}

std::int32_t H0Device::motor_2_winding_counts() const {
    if (!baselines_valid_) {
        return 0;
    }

    return winding_count_from_raw(raw_.motor_2_raw_count, motor_2_baseline_count_);
}

SafetySample H0Device::safety_sample() const {
    const double time_s = static_cast<double>(raw_.time_us) * 1e-6;

    return SafetySample{
        .time_s = time_s,

        .joint_angle_rad = joint_angle_rad_from_raw(raw_.qraw),

        .winding = {
            .motor_1_m = static_cast<double>(motor_1_winding_counts()) * meters_per_motor_count,

            .motor_2_m = static_cast<double>(motor_2_winding_counts()) * meters_per_motor_count}};
}

void H0Device::ingest_sample(const RawHardwareSample& sample) {
    raw_ = sample;
    have_sample_ = true;

    if (!baselines_valid_) {
        return;
    }

    safety_.update(safety_sample());
}

CommandDisposition H0Device::handle_command(const HostCommand& command) {
    switch (command.type) {

    case CommandType::arm: {
        if (!have_sample_) {
            return CommandDisposition::rejected_no_sample;
        }

        motor_1_baseline_count_ = raw_.motor_1_raw_count;

        motor_2_baseline_count_ = raw_.motor_2_raw_count;

        motor_1_target_winding_counts_ = 0;
        motor_2_target_winding_counts_ = 0;

        baselines_valid_ = true;

        if (!safety_.arm(safety_sample())) {
            baselines_valid_ = false;

            return CommandDisposition::rejected_by_safety;
        }

        return CommandDisposition::accepted;
    }

    case CommandType::disable:
        safety_.disable();

        baselines_valid_ = false;

        motor_1_target_winding_counts_ = 0;
        motor_2_target_winding_counts_ = 0;

        return CommandDisposition::accepted;

    case CommandType::clear_fault:
        safety_.clear_fault();

        baselines_valid_ = false;

        motor_1_target_winding_counts_ = 0;
        motor_2_target_winding_counts_ = 0;

        return CommandDisposition::accepted;

    case CommandType::move: {
        if (motion_authority_ == MotionAuthority::inhibited) {
            return CommandDisposition::rejected_motion_inhibited;
        }

        if (!have_sample_ || !baselines_valid_) {
            return CommandDisposition::rejected_no_sample;
        }

        /*
         * Phase 1:
         *
         * at most one motor may require motion.
         */
        const auto candidate =
            single_motor_drive_command(motor_1_winding_counts(), motor_2_winding_counts(),

                                       command.motor_1_target_winding_counts,

                                       command.motor_2_target_winding_counts);

        if (!candidate.has_value()) {
            return CommandDisposition::rejected_coordinated_motion;
        }

        const MotionTarget target{
            .winding = {.motor_1_m = static_cast<double>(command.motor_1_target_winding_counts) *
                                     meters_per_motor_count,

                        .motor_2_m = static_cast<double>(command.motor_2_target_winding_counts) *
                                     meters_per_motor_count}};

        if (!safety_.begin_motion(target, safety_sample())) {
            return CommandDisposition::rejected_by_safety;
        }

        motor_1_target_winding_counts_ = command.motor_1_target_winding_counts;

        motor_2_target_winding_counts_ = command.motor_2_target_winding_counts;

        return CommandDisposition::accepted;
    }
    }

    return CommandDisposition::rejected_by_safety;
}

std::optional<MotorDriveCommand> H0Device::drive_command() const {
    /*
     * SafetySupervisor allows drive authority only in
     * MOVING or HOLDING.
     *
     * HOLDING remains active so a small disturbance can be
     * corrected before it grows into a tracking fault.
     */
    if (!safety_.driver_enable_allowed()) {
        return MotorDriveCommand{.motor_1 = MotorDriveDirection::off,

                                 .motor_2 = MotorDriveDirection::off};
    }

    return single_motor_drive_command(motor_1_winding_counts(), motor_2_winding_counts(),

                                      motor_1_target_winding_counts_,
                                      motor_2_target_winding_counts_);
}

TelemetryPacket H0Device::make_telemetry() {
    TelemetryPacket packet{};

    packet.sequence = telemetry_sequence_++;

    if (!have_sample_) {
        return packet;
    }

    packet.time_us = raw_.time_us;

    packet.qraw = raw_.qraw;

    packet.motor_1_raw_count = raw_.motor_1_raw_count;

    packet.motor_2_raw_count = raw_.motor_2_raw_count;

    if (baselines_valid_) {
        packet.motor_1_baseline_count = motor_1_baseline_count_;

        packet.motor_2_baseline_count = motor_2_baseline_count_;

        packet.motor_1_winding_counts = motor_1_winding_counts();

        packet.motor_2_winding_counts = motor_2_winding_counts();

        packet.motor_1_target_winding_counts = motor_1_target_winding_counts_;

        packet.motor_2_target_winding_counts = motor_2_target_winding_counts_;
    }

    packet.state = safety_.state();

    packet.fault = safety_.fault();

    /*
     * Telemetry is emitted only after each short drive pulse
     * has ended, so the physical enable pins are LOW here.
     */
    packet.driver_enabled = false;

    return packet;
}

} // namespace atj::h0
