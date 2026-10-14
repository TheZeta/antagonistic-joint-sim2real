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

    std::uint64_t last_telemetry_time_us = 0;
    constexpr std::uint64_t telemetry_period_us = 20'000;

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

        const auto servo = device.servo_output();

        if (!h0_firmware::apply_servo_output(servo)) {
            h0_firmware::force_disable_drivers();
        }

        const bool motor_1_enabled = servo.motor_1.mode != atj::h0::MotorServoMode::coast;
        const bool motor_2_enabled = servo.motor_2.mode != atj::h0::MotorServoMode::coast;
        const bool driver_enabled = motor_1_enabled || motor_2_enabled;

        const bool telemetry_due = last_telemetry_time_us == 0 ||
                                   sample->time_us - last_telemetry_time_us >= telemetry_period_us;

        if (telemetry_due) {
            const auto telemetry = device.make_telemetry(driver_enabled);
            h0_firmware::write_serial_line(atj::h0::serialize_telemetry(telemetry));
            last_telemetry_time_us = sample->time_us;
        }

        h0_firmware::wait_for_next_cycle();
    }
}
