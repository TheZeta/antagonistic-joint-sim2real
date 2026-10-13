#include "board.hpp"

#include "atj/h0_device.hpp"
#include "atj/h0_limits.hpp"
#include "atj/h0_protocol.hpp"

extern "C" void H0_Powered_PreInit() {
    h0_firmware::initialize_board();
}

extern "C" void H0_Powered_Run() {
    h0_firmware::force_disable_drivers();
    h0_firmware::start_command_receiver();

    atj::h0::H0Device device(atj::h0::h0_safety_limits(), atj::h0::MotionAuthority::coordinated);

    while (true) {
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

        const auto drive = device.drive_command();
        bool driver_active = false;

        if (!drive.has_value()) {
            h0_firmware::force_disable_drivers();
        } else {
            const bool motor_1_active = drive->motor_1 != atj::h0::MotorDriveDirection::off;
            const bool motor_2_active = drive->motor_2 != atj::h0::MotorDriveDirection::off;

            if (!h0_firmware::apply_drive_command(*drive)) {
                h0_firmware::force_disable_drivers();
            } else {
                driver_active = motor_1_active || motor_2_active;
            }
        }
        /*
         * apply_drive_command() has already returned all
         * physical outputs to LOW before telemetry.
         */
        const auto telemetry = device.make_telemetry(driver_active);

        h0_firmware::write_serial_line(atj::h0::serialize_telemetry(telemetry));
        h0_firmware::wait_for_next_cycle();
    }
}
