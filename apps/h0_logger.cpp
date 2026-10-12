#include "atj/h0_protocol.hpp"
#include "atj/h0_telemetry.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void handle_signal(int) {
    keep_running = 0;
}

int open_serial(const char* device) {
    const int fd = ::open(device, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        throw std::runtime_error(std::string("Failed to open serial port: ") +
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
     * Block until at least one byte arrives.
     */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);

        throw ::std::runtime_error("tcsetattr failed");
    }

    tcflush(fd, TCIFLUSH);

    return fd;
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

} // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <serial-device> <output.csv>\n";

        return 1;
    }

    std::signal(SIGINT, handle_signal);

    const int serial_fd = open_serial(argv[1]);

    std::ofstream output{argv[2]};

    if (!output) {
        ::close(serial_fd);

        std::cerr << "Failed to open output file\n";

        return 1;
    }

    output << std::setprecision(12);

    write_header(output);

    std::string line;
    line.reserve(256);

    std::size_t samples = 0;
    std::size_t rejected_lines = 0;

    while (keep_running) {
        char byte = '\0';

        const ssize_t result = ::read(serial_fd, &byte, 1);

        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "Serial read failed: " << std::strerror(errno) << '\n';

            break;
        }

        if (result == 0) {
            continue;
        }

        if (byte == '\r') {
            continue;
        }

        if (byte != '\n') {
            if (line.size() < 512) {
                line.push_back(byte);
            } else {
                line.clear();
                ++rejected_lines;
            }

            continue;
        }

        if (line.empty()) {
            continue;
        }

        line.push_back('\n');

        const auto packet = atj::h0::parse_telemetry(line);

        line.clear();

        if (!packet.has_value()) {
            ++rejected_lines;
            continue;
        }

        write_sample(output, atj::h0::physical_telemetry(*packet));

        ++samples;
    }

    output.flush();

    ::close(serial_fd);

    std::cerr << "\nLogged " << samples << " telemetry samples";

    if (rejected_lines != 0) {
        std::cerr << " (" << rejected_lines << " rejected lines)";
    }

    std::cerr << '\n';

    return 0;
}
