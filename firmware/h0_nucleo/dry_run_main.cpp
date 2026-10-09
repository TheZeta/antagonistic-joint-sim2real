#include "board.hpp"

#include "atj/h0_device.hpp"
#include "atj/h0_protocol.hpp"

namespace {

/*
 * DO NOT fill these from the old unit-test fixture.
 *
 * This function will receive the real H0 safety envelope
 * before we flash the permanent motion-capable firmware.
 *
 * For the dry-run phase we only need values large enough
 * to validate the commissioned observation region, but
 * they must still be explicitly chosen and documented.
 */
atj::h0::SafetyLimits h0_safety_limits();

} // namespace

int main() {
    h0_firmware::initialize_board();

    /*
     * First executable action after board initialization:
     * force both drivers inactive.
     */
    h0_firmware::force_disable_drivers();

    atj::h0::H0DryRunDevice device(h0_safety_limits());

    while (true) {
        /*
         * Dry-run invariant:
         *
         * Every iteration reinforces driver-disable.
         */
        h0_firmware::force_disable_drivers();

        const auto sample = h0_firmware::read_hardware_sample();

        device.ingest_sample(sample);

        while (true) {
            const auto line = h0_firmware::try_read_command_line();

            if (!line.has_value()) {
                break;
            }

            const auto command = atj::h0::parse_command(*line);

            if (!command.has_value()) {
                continue;
            }

            /*
             * MOVE is parsed but rejected internally by
             * H0DryRunDevice
             */
            [[maybe_unused]]
            const auto disposition = device.handle_command(*command);
        }

        const auto telemetry = device.make_telemetry();

        h0_firmware::write_serial_line(atj::h0::serialize_telemetry(telemetry));

        /*
         * Belt and suspenders.
         */
        h0_firmware::force_disable_drivers();

        h0_firmware::wait_for_next_cycle();
    }
}
