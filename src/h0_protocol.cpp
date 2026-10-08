#include "atj/h0_protocol.hpp"

#include <charconv>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace atj::h0 {

namespace {

std::vector<std::string_view> split_csv(std::string_view line) {
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
        line.remove_suffix(1);
    }

    std::vector<std::string_view> fields;

    std::size_t begin = 0;

    while (begin <= line.size()) {
        const std::size_t comma = line.find(',', begin);

        if (comma == std::string_view::npos) {
            fields.emplace_back(line.substr(begin));

            break;
        }

        fields.emplace_back(line.substr(begin, comma - begin));

        begin = comma + 1;
    }

    return fields;
}

template <typename T> bool parse_integer(std::string_view text, T& value) {
    if (text.empty()) {
        return false;
    }

    const char* begin = text.data();
    const char* end = text.data() + text.size();

    const auto result = std::from_chars(begin, end, value);

    return result.ec == std::errc{} && result.ptr == end;
}

bool valid_state_code(int value) {
    return value >= static_cast<int>(SafetyState::disabled) &&
           value <= static_cast<int>(SafetyState::fault);
}

bool valid_fault_code(int value) {
    return value >= static_cast<int>(SafetyFault::none) &&
           value <= static_cast<int>(SafetyFault::motion_timeout);
}

} // namespace

std::int32_t winding_count_from_raw(std::int32_t raw_count, std::int32_t baseline_count) {
    return baseline_count - raw_count;
}

std::int32_t raw_target_from_winding_count(std::int32_t baseline_count,
                                           std::int32_t winding_target_count) {
    return baseline_count - winding_target_count;
}

std::string serialize_command(const HostCommand& command) {
    std::ostringstream output;

    output << "H0C," << protocol_version << ',' << command.sequence << ',';

    switch (command.type) {
    case CommandType::arm:
        output << "ARM";
        break;

    case CommandType::move:
        output << "MOVE," << command.motor_1_target_winding_counts << ','
               << command.motor_2_target_winding_counts;
        break;

    case CommandType::disable:
        output << "DISABLE";
        break;

    case CommandType::clear_fault:
        output << "CLEAR_FAULT";
        break;
    }

    output << '\n';

    return output.str();
}

std::optional<HostCommand> parse_command(std::string_view line) {
    const auto fields = split_csv(line);

    if (fields.size() < 4) {
        return std::nullopt;
    }

    if (fields[0] != "H0C") {
        return std::nullopt;
    }

    int version = 0;

    if (!parse_integer(fields[1], version) || version != protocol_version) {
        return std::nullopt;
    }

    std::uint32_t sequence = 0;

    if (!parse_integer(fields[2], sequence)) {
        return std::nullopt;
    }

    if (fields[3] == "ARM") {
        if (fields.size() != 4) {
            return std::nullopt;
        }

        return HostCommand{.sequence = sequence, .type = CommandType::arm};
    }

    if (fields[3] == "DISABLE") {
        if (fields.size() != 4) {
            return std::nullopt;
        }

        return HostCommand{.sequence = sequence, .type = CommandType::disable};
    }

    if (fields[3] == "CLEAR_FAULT") {
        if (fields.size() != 4) {
            return std::nullopt;
        }

        return HostCommand{.sequence = sequence, .type = CommandType::clear_fault};
    }

    if (fields[3] == "MOVE") {
        if (fields.size() != 6) {
            return std::nullopt;
        }

        std::int32_t target_1 = 0;
        std::int32_t target_2 = 0;

        if (!parse_integer(fields[4], target_1) || !parse_integer(fields[5], target_2)) {
            return std::nullopt;
        }

        return HostCommand{.sequence = sequence,
                           .type = CommandType::move,
                           .motor_1_target_winding_counts = target_1,
                           .motor_2_target_winding_counts = target_2};
    }

    return std::nullopt;
}

std::string serialize_telemetry(const TelemetryPacket& packet) {
    std::ostringstream output;

    output << "H0T," << protocol_version << ',' << packet.sequence << ',' << packet.time_us << ','
           << packet.qraw << ',' << packet.motor_1_raw_count << ',' << packet.motor_2_raw_count
           << ',' << packet.motor_1_baseline_count << ',' << packet.motor_2_baseline_count << ','
           << packet.motor_1_winding_counts << ',' << packet.motor_2_winding_counts << ','
           << packet.motor_1_target_winding_counts << ',' << packet.motor_2_target_winding_counts
           << ',' << static_cast<int>(packet.state) << ',' << static_cast<int>(packet.fault) << ','
           << (packet.driver_enabled ? 1 : 0) << '\n';

    return output.str();
}

std::optional<TelemetryPacket> parse_telemetry(std::string_view line) {
    const auto fields = split_csv(line);

    if (fields.size() != 16) {
        return std::nullopt;
    }

    if (fields[0] != "H0T") {
        return std::nullopt;
    }

    int version = 0;

    if (!parse_integer(fields[1], version) || version != protocol_version) {
        return std::nullopt;
    }

    TelemetryPacket packet{};

    int state_code = 0;
    int fault_code = 0;
    int driver_code = 0;

    if (!parse_integer(fields[2], packet.sequence) || !parse_integer(fields[3], packet.time_us) ||
        !parse_integer(fields[4], packet.qraw) ||
        !parse_integer(fields[5], packet.motor_1_raw_count) ||
        !parse_integer(fields[6], packet.motor_2_raw_count) ||
        !parse_integer(fields[7], packet.motor_1_baseline_count) ||
        !parse_integer(fields[8], packet.motor_2_baseline_count) ||
        !parse_integer(fields[9], packet.motor_1_winding_counts) ||
        !parse_integer(fields[10], packet.motor_2_winding_counts) ||
        !parse_integer(fields[11], packet.motor_1_target_winding_counts) ||
        !parse_integer(fields[12], packet.motor_2_target_winding_counts) ||
        !parse_integer(fields[13], state_code) || !parse_integer(fields[14], fault_code) ||
        !parse_integer(fields[15], driver_code)) {
        return std::nullopt;
    }

    if (!valid_state_code(state_code) || !valid_fault_code(fault_code) ||
        (driver_code != 0 && driver_code != 1)) {
        return std::nullopt;
    }

    if (packet.qraw < 0 || packet.qraw >= 4096) {
        return std::nullopt;
    }

    packet.state = static_cast<SafetyState>(state_code);
    packet.fault = static_cast<SafetyFault>(fault_code);

    packet.driver_enabled = driver_code == 1;

    return packet;
}

} // namespace atj::h0
