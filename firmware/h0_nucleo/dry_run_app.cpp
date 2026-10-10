#include "board.hpp"

#include "atj/h0_device.hpp"
#include "atj/h0_protocol.hpp"

namespace {

atj::h0::SafetyLimits h0_dry_run_limits() {
    return atj::h0::SafetyLimits{.max_abs_joint_angle_rad = 1.0471975512,
                                 .max_abs_motor_winding_m = 0.050,
                                 .max_command_step_m = 0.005,
                                 .direction_movement_threshold_m = 0.0002,
                                 .direction_check_timeout_s = 0.050,
                                 .tracking_error_grace_s = 0.100,
                                 .max_tracking_error_m = 0.002,
                                 .target_tolerance_m = 0.0002,
                                 .max_motion_duration_s = 0.500};
}

} // namespace

extern "C" void H0_DryRun_PreInit() {
    h0_firmware::initialize_board();
}

extern "C" void H0_DryRun_Run() {
    h0_firmware::force_disable_drivers();
    h0_firmware::start_command_receiver();

    atj::h0::H0DryRunDevice device(h0_dry_run_limits());

    while (true) {
        h0_firmware::force_disable_drivers();

        const auto sample = h0_firmware::try_read_hardware_sample();

        if (!sample.has_value()) {
            h0_firmware::force_disable_drivers();
            h0_firmware::wait_for_next_cycle();

            continue;
        }

        device.ingest_sample(*sample);

        while (true) {
            const auto line = h0_firmware::try_read_command_line();

            if (!line.has_value()) {
                break;
            }

            const auto command = atj::h0::parse_command(*line);

            if (!command.has_value()) {
                continue;
            }

            [[maybe_unused]]
            const auto disposition = device.handle_command(*command);
        }

        const auto telemetry = device.make_telemetry();

        h0_firmware::write_serial_line(atj::h0::serialize_telemetry(telemetry));
        h0_firmware::force_disable_drivers();
        h0_firmware::wait_for_next_cycle();
    }
}
