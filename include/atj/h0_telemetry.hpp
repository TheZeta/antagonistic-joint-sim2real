#pragma once

#include "atj/h0_calibration.hpp"
#include "atj/h0_protocol.hpp"

#include <cstdint>
#include <numbers>

namespace atj::h0 {

struct PhysicalTelemetry {
    std::uint32_t sequence;
    double time_s;

    int qraw;
    double q_rad;
    double q_deg;

    std::int32_t motor_1_raw_count;
    std::int32_t motor_2_raw_count;

    std::int32_t motor_1_baseline_count;
    std::int32_t motor_2_baseline_count;

    std::int32_t motor_1_winding_counts;
    std::int32_t motor_2_winding_counts;

    double motor_1_winding_m;
    double motor_2_winding_m;

    double common_winding_m;
    double differential_winding_m;

    std::int32_t motor_1_target_counts;
    std::int32_t motor_2_target_counts;

    double motor_1_target_m;
    double motor_2_target_m;

    SafetyState state;
    SafetyFault fault;

    bool driver_enabled;
};

[[nodiscard]]
inline PhysicalTelemetry physical_telemetry(const TelemetryPacket& packet) {
    const double motor_1_winding_m =
        static_cast<double>(packet.motor_1_winding_counts) * meters_per_motor_count;
    const double motor_2_winding_m =
        static_cast<double>(packet.motor_2_winding_counts) * meters_per_motor_count;

    const double q_rad = joint_angle_rad_from_raw(packet.qraw);

    return PhysicalTelemetry{
        .sequence = packet.sequence,
        .time_s = static_cast<double>(packet.time_us) * 1e-6,

        .qraw = packet.qraw,
        .q_rad = q_rad,
        .q_deg = q_rad * 180.0 / std::numbers::pi,

        .motor_1_raw_count = packet.motor_1_raw_count,
        .motor_2_raw_count = packet.motor_2_raw_count,

        .motor_1_baseline_count = packet.motor_1_baseline_count,
        .motor_2_baseline_count = packet.motor_2_baseline_count,

        .motor_1_winding_counts = packet.motor_1_winding_counts,
        .motor_2_winding_counts = packet.motor_2_winding_counts,

        .motor_1_winding_m = motor_1_winding_m,
        .motor_2_winding_m = motor_2_winding_m,

        .common_winding_m = 0.5 * (motor_1_winding_m + motor_2_winding_m),

        /*
         * Positive differential corresponds to q+:
         *
         * Motor 2 winding - Motor 1 winding.
         */
        .differential_winding_m = motor_2_winding_m - motor_1_winding_m,

        .motor_1_target_counts = packet.motor_1_target_winding_counts,
        .motor_2_target_counts = packet.motor_2_target_winding_counts,

        .motor_1_target_m =
            static_cast<double>(packet.motor_1_target_winding_counts) * meters_per_motor_count,
        .motor_2_target_m =
            static_cast<double>(packet.motor_2_target_winding_counts) * meters_per_motor_count,

        .state = packet.state,
        .fault = packet.fault,

        .driver_enabled = packet.driver_enabled};
}

} // namespace atj::h0
