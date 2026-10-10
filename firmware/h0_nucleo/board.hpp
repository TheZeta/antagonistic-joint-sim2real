#pragma once

#include "atj/h0_device.hpp"

#include <optional>
#include <string>

namespace h0_firmware {

/*
 * Initialize clocks, GPIO, UART/USB serial,
 * AS5600 interface and motor encoders.
 *
 * Motor drivers MUST remain disabled.
 */
void initialize_board();

/*
 * Hardware-level emergency primitive.
 *
 * Both IBT-2 driver enable lines must be force inactive.
 */
void force_disable_drivers();

void start_command_receiver();

/*
 * Read one coherent sensor snapshot.
 */
[[nodiscard]]
std::optional<atj::h0::RawHardwareSample> try_read_hardware_sample();

/*
 * Return one complete received serial line when available.
 */
[[nodiscard]]
std::optional<std::string> try_read_command_line();

/*
 * Transmit one protocol line
 */
void write_serial_line(const std::string& line);

/*
 * Maintain the chosen telemetry/control-loop period.
 */
void wait_for_next_cycle();

} // namespace h0_firmware
