#pragma once

#include "atj/h0_safety.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace atj::h0 {

inline constexpr int protocol_version = 1;

enum class CommandType { arm = 1, move = 2, disable = 3, clear_fault = 4 };

struct HostCommand {
    std::uint32_t sequence;
    CommandType type;

    /*
     * Used only by MOVE.
     *
     * Absolute normalized winding-count targets,
     * relative to baselines captured at ARM.
     *
     * Positive -> WIND
     * Negative -> UNWIND
     */
    std::int32_t motor_1_target_winding_counts = 0;
    std::int32_t motor_2_target_winding_counts = 0;
};

struct TelemetryPacket {
    std::uint32_t sequence;
    std::uint64_t time_us;

    int qraw;

    std::int32_t motor_1_raw_count;
    std::int32_t motor_2_raw_count;

    std::int32_t motor_1_baseline_count;
    std::int32_t motor_2_baseline_count;

    /*
     * Normalized measured winding counts:
     *
     * baseline - raw
     *
     * Positive -> WIND.
     */
    std::int32_t motor_1_winding_counts;
    std::int32_t motor_2_winding_counts;

    std::int32_t motor_1_target_winding_counts;
    std::int32_t motor_2_target_winding_counts;

    SafetyState state;
    SafetyFault fault;

    bool driver_enabled;
};

[[nodiscard]]
std::string serialize_command(const HostCommand& command);

[[nodiscard]]
std::optional<HostCommand> parse_command(std::string_view line);

[[nodiscard]]
std::string serialize_telemetry(const TelemetryPacket& packet);

[[nodiscard]]
std::optional<TelemetryPacket> parse_telemetry(std::string_view line);

/*
 * Raw motor count decreases during WIND.
 *
 * Therefore:
 *
 * normalized winding count = baseline -raw
 */
[[nodiscard]]
std::int32_t winding_count_from_raw(std::int32_t raw_count, std::int32_t baseline_count);

/*
 * Inverse mapping:
 *
 * raw target = baseline - winding target
 */
[[nodiscard]]
std::int32_t raw_target_from_winding_count(std::int32_t baseline_count,
                                           std::int32_t winding_target_count);

} // namespace atj::h0
