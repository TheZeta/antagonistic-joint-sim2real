#include "atj/h0_device.hpp"

#include "atj/h0_calibration.hpp"

namespace atj::h0 {

H0DryRunDevice::H0DryRunDevice(SafetyLimits limits) : safety_(limits) {}

std::int32_t H0DryRunDevice::motor_1_winding_counts() const {
    if (!baselines_valid_) {
        return 0;
    }

    return winding_count_from_raw(raw_.motor_1_raw_count, motor_1_baseline_count_);
}

std::int32_t H0DryRunDevice::motor_2_winding_counts() const {
    if (!baselines_valid_) {
        return 0;
    }

    return winding_count_from_raw(raw_.motor_2_raw_count, motor_2_baseline_count_);
}

SafetySample H0DryRunDevice::safety_sample() const {
    const double time_s = static_cast<double>(raw_.time_us) * 1e-6;

    const double q = joint_angle_rad_from_raw(raw_.qraw);

    const double winding_1_m =
        static_cast<double>(motor_1_winding_counts()) * meters_per_motor_count;

    const double winding_2_m =
        static_cast<double>(motor_2_winding_counts()) * meters_per_motor_count;

    return SafetySample{.time_s = time_s,
                        .joint_angle_rad = q,
                        .winding = {.motor_1_m = winding_1_m, .motor_2_m = winding_2_m}};
}

void H0DryRunDevice::ingest_sample(const RawHardwareSample& sample) {
    raw_ = sample;
    have_sample_ = true;

    /*
     * Before ARM there is no motor-coordinate baseline,
     * so there is nothing meaningful for the safety
     * supervisor to monitor beyond startup observation.
     */
    if (!baselines_valid_) {
        return;
    }

    safety_.update(safety_sample());
}

CommandDisposition H0DryRunDevice::handle_command(const HostCommand& command) {
    switch (command.type) {

    case CommandType::arm: {
        if (!have_sample_) {
            return CommandDisposition::rejected_no_sample;
        }

        /*
         * ARM defines the current physical motor positions
         * as zero winding displacement.
         */
        motor_1_baseline_count_ = raw_.motor_1_raw_count;
        motor_2_baseline_count_ = raw_.motor_2_raw_count;

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

        return CommandDisposition::accepted;

    case CommandType::clear_fault:
        safety_.clear_fault();
        baselines_valid_ = false;

        return CommandDisposition::accepted;

    case CommandType::move:
        /*
         * Intentionally impossible in dry-run firmware.
         *
         * We parse MOVE so protocol compatibility can be
         * tested, but we do not pass it to begin_motion().
         */
        return CommandDisposition::rejected_motion_inhibited;
    }

    return CommandDisposition::rejected_by_safety;
}

TelemetryPacket H0DryRunDevice::make_telemetry() {
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
    }

    /*
     * There are deliberately no active motor targets
     * in dry-run firmware.
     */
    packet.motor_1_target_winding_counts = 0;
    packet.motor_2_target_winding_counts = 0;

    packet.state = safety_.state();
    packet.fault = safety_.fault();

    /*
     * Critical invariant:
     *
     * dry-run firmware physically cannot enable drivers.
     */
    packet.driver_enabled = false;

    return packet;
}

} // namespace atj::h0
