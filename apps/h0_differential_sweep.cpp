#include "atj/h0_calibration.hpp"
#include "atj/h0_limits.hpp"
#include "atj/h0_protocol.hpp"
#include "atj/h0_telemetry.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

volatile std::sig_atomic_t keep_running = 1;

constexpr std::int32_t default_preload_counts = 0;

void handle_signal(int) {
    keep_running = 0;
}

int open_serial(const char* device) {
    const int fd = ::open(device, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        throw std::runtime_error(std::string{"Failed to open serial port: "} +
                                 std::strerror(errno));
    }

    termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        throw std::runtime_error("tcgetattr failed");
    }

    cfmakeraw(&tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CRTSCTS;

    /*
     * Return from read at least every 0.1 s.
     */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);

        throw std::runtime_error("tcsetattr failed");
    }

    tcflush(fd, TCIFLUSH);

    return fd;
}

void write_all(int fd, std::string_view text) {
    std::size_t written = 0;

    while (written < text.size()) {
        const ssize_t result = ::write(fd, text.data() + written, text.size() - written);

        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }

            throw std::runtime_error("Serial write failed");
        }

        written += static_cast<std::size_t>(result);
    }
}

void send_command(int fd, const atj::h0::HostCommand& command) {
    write_all(fd, atj::h0::serialize_command(command));
}

std::optional<atj::h0::TelemetryPacket> read_packet(int fd, std::string& line) {
    char byte = '\0';

    const ssize_t result = ::read(fd, &byte, 1);

    if (result < 0) {
        if (errno == EINTR) {
            return std::nullopt;
        }

        throw std::runtime_error("Serial read failed");
    }

    if (result == 0) {
        return std::nullopt;
    }

    if (byte == '\r') {
        return std::nullopt;
    }

    if (byte != '\n') {
        if (line.size() < 512) {
            line.push_back(byte);
        } else {
            line.clear();
        }

        return std::nullopt;
    }

    if (line.empty()) {
        return std::nullopt;
    }

    line.push_back('\n');

    const auto packet = atj::h0::parse_telemetry(line);

    line.clear();

    return packet;
}

void write_header(std::ofstream& output) {
    output << "sequence,"
           << "time_s,"
           << "qraw,"
           << "q_rad,"
           << "q_deg,"
           << "m1_raw_count,"
           << "m2_raw_count,"
           << "m1_baseline_count,"
           << "m2_baseline_count,"
           << "m1_winding_counts,"
           << "m2_winding_counts,"
           << "m1_winding_m,"
           << "m2_winding_m,"
           << "common_winding_m,"
           << "differential_winding_m,"
           << "m1_target_counts,"
           << "m2_target_counts,"
           << "m1_target_m,"
           << "m2_target_m,"
           << "state,"
           << "fault,"
           << "driver_enabled\n";
}

void write_sample(std::ofstream& output, const atj::h0::PhysicalTelemetry& sample) {
    output << sample.sequence << ',' << sample.time_s << ',' << sample.qraw << ',' << sample.q_rad
           << ',' << sample.q_deg << ',' << sample.motor_1_raw_count << ','
           << sample.motor_2_raw_count << ',' << sample.motor_1_baseline_count << ','
           << sample.motor_2_baseline_count << ',' << sample.motor_1_winding_counts << ','
           << sample.motor_2_winding_counts << ',' << sample.motor_1_winding_m << ','
           << sample.motor_2_winding_m << ',' << sample.common_winding_m << ','
           << sample.differential_winding_m << ',' << sample.motor_1_target_counts << ','
           << sample.motor_2_target_counts << ',' << sample.motor_1_target_m << ','
           << sample.motor_2_target_m << ',' << static_cast<int>(sample.state) << ','
           << static_cast<int>(sample.fault) << ',' << (sample.driver_enabled ? 1 : 0) << '\n';
}

atj::h0::TelemetryPacket wait_for_state(int fd, std::ofstream& output,
                                        atj::h0::SafetyState desired_state, double timeout_s,
                                        bool abort_on_fault = true) {
    using Clock = std::chrono::steady_clock;

    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);

    std::string line;

    while (keep_running && Clock::now() < deadline) {
        const auto packet = read_packet(fd, line);

        if (!packet.has_value()) {
            continue;
        }

        write_sample(output, atj::h0::physical_telemetry(*packet));

        if (abort_on_fault && packet->fault != atj::h0::SafetyFault::none) {
            throw std::runtime_error("H0 safety fault " +
                                     std::to_string(static_cast<int>(packet->fault)));
        }

        if (packet->state == desired_state) {
            return *packet;
        }
    }

    if (!keep_running) {
        throw std::runtime_error("Experiment interrupted");
    }

    throw std::runtime_error("Timed out waiting for H0 state");
}

void dwell(int fd, std::ofstream& output, double duration_s) {
    using Clock = std::chrono::steady_clock;

    const auto deadline = Clock::now() + std::chrono::duration<double>(duration_s);

    std::string line;

    while (keep_running && Clock::now() < deadline) {
        const auto packet = read_packet(fd, line);

        if (!packet.has_value()) {
            continue;
        }

        write_sample(output, atj::h0::physical_telemetry(*packet));

        if (packet->fault != atj::h0::SafetyFault::none) {
            throw std::runtime_error("H0 safety fault " +
                                     std::to_string(static_cast<int>(packet->fault)));
        }
    }

    if (!keep_running) {
        throw std::runtime_error("Experiment interrupted");
    }
}

void perform_move(int fd, std::ofstream& output, std::uint32_t& command_sequence,
                  std::int32_t motor_1_target, std::int32_t motor_2_target) {
    std::cout << "MOVE " << motor_1_target << ", " << motor_2_target << '\n';

    send_command(fd, atj::h0::HostCommand{.sequence = command_sequence++,
                                          .type = atj::h0::CommandType::move,
                                          .motor_1_target_winding_counts = motor_1_target,
                                          .motor_2_target_winding_counts = motor_2_target});

    wait_for_state(fd, output, atj::h0::SafetyState::holding, 3.0);

    /*
     * Characterization plateau.
     */
    dwell(fd, output, 1.0);
}

} // namespace

int main(int argc, char** argv) {
    if (!(argc == 3 || argc == 4)) {
        std::cerr << "Usage: " << argv[0] << " <serial-device> <output.csv> [preload-counts]\n";

        return 1;
    }

    std::int32_t preload_counts = default_preload_counts;

    if (argc == 4) {
        try {
            preload_counts = static_cast<std::int32_t>(std::stol(argv[3]));
        } catch (...) {
            std::cerr << "Invalid preload count\n";

            return 1;
        }
    }

    if (preload_counts < 0) {
        std::cerr << "Preload count must be non-negative\n";

        return 1;
    }

    const double preload_m = static_cast<double>(preload_counts) * atj::h0::meters_per_motor_count;

    std::cout << "Common-mode preload: " << preload_counts << " counts (" << preload_m * 1000.0
              << " mm tendon winding per motor)\n";

    constexpr std::int32_t maximum_differential_offset_counts = 30;
    const auto maximum_requested_counts = preload_counts + maximum_differential_offset_counts;
    const double maximum_requested_m =
        static_cast<double>(maximum_requested_counts) * atj::h0::meters_per_motor_count;

    if (maximum_requested_m > atj::h0::h0_max_abs_motor_winding_m) {
        std::cerr << "Preload + sweep amplitude exceeds H0 winding safety limit\n";

        return 1;
    }

    std::signal(SIGINT, handle_signal);

    int fd = -1;
    std::uint32_t command_sequence = 1;

    try {
        fd = open_serial(argv[1]);
        std::ofstream output{argv[2]};

        if (!output) {
            throw std::runtime_error("Failed to open output CSV");
        }

        output << std::setprecision(12);

        write_header(output);

        /*
         * Force a known startup state.
         *
         * DISABLE handles moving/holding.
         * CLEAR_FAULT handles a previously latched fault.
         */
        send_command(fd, {.sequence = command_sequence++, .type = atj::h0::CommandType::disable});
        send_command(fd,
                     {.sequence = command_sequence++, .type = atj::h0::CommandType::clear_fault});

        wait_for_state(fd, output, atj::h0::SafetyState::disabled, 2.0, false);

        /*
         * ARM establishes the encoder baselines.
         */
        send_command(fd, {.sequence = command_sequence++, .type = atj::h0::CommandType::arm});

        wait_for_state(fd, output, atj::h0::SafetyState::armed, 2.0);

        /*
         * Establish the experiment's common-mode preload.
         *
         * Positive normalized winding count means WIND.
         *
         * Both motors receive the same target, so ideally this
         * changes tendon preload without commanding differential
         * joint motion.
         */
        if (preload_counts > 0) {
            std::cout << "PRELOAD " << preload_counts << ", " << preload_counts << '\n';

            perform_move(fd, output, command_sequence, preload_counts, preload_counts);
        } else {
            /*
             * Record the initial neutral condition even when no
             * automatic preload was requested.
             */
            dwell(fd, output, 1.0);
        }

        const std::vector<std::int32_t> differential_offsets{5,   10,  15,  20,

                                                             15,  10,  5,   0,

                                                             -5,  -10, -15, -20,

                                                             -15, -10, -5,  0};

        for (const std::int32_t offset : differential_offsets) {
            /*
             * Our established differential convention:
             *
             * q+:
             *   Motor 1 UNWIND
             *   Motor 2 WIND
             *
             * therefore:
             *
             * M1 = preload - offset
             * M2 = preload + offset
             */
            const ::std::int32_t motor_1_target = preload_counts - offset;
            const ::std::int32_t motor_2_target = preload_counts + offset;

            perform_move(fd, output, command_sequence, motor_1_target, motor_2_target);
        }

        std::cout << "RETURN TO PRELOAD " << preload_counts << ", " << preload_counts << '\n';

        perform_move(fd, output, command_sequence, preload_counts, preload_counts);

        send_command(fd, {.sequence = command_sequence++, .type = atj::h0::CommandType::disable});

        wait_for_state(fd, output, atj::h0::SafetyState::disabled, 2.0);

        output.flush();
        ::close(fd);
        std::cout << "Differential sweep complete.\n";

        return 0;
    } catch (const std::exception& error) {
        /*
         * Best-effort emergency shutdown.
         */
        if (fd >= 0) {
            try {
                send_command(
                    fd, {.sequence = command_sequence++, .type = atj::h0::CommandType::disable});
            } catch (...) {
            }

            ::close(fd);
        }

        std::cerr << "Sweep aborted: " << error.what() << '\n';

        return 1;
    }
}
