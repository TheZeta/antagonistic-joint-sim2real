#pragma once

#include "atj/h0_motor_servo.hpp"
#include "atj/h0_protocol.hpp"

#include <cstdint>
#include <optional>

namespace atj::h0 {

struct RawHardwareSample {
    std::uint64_t time_us;

    int qraw;

    std::int32_t motor_1_raw_count;
    std::int32_t motor_2_raw_count;
};

enum class CommandDisposition {
    accepted,
    rejected_no_sample,
    rejected_motion_inhibited,
    rejected_by_safety
};

enum class MotionAuthority { inhibited, coordinated };

class H0Device {
  public:
    explicit H0Device(SafetyLimits limits,
                      MotionAuthority motion_authority = MotionAuthority::inhibited);

    /*
     * Supply the newest physical sensor readings.
     */
    void ingest_sample(const RawHardwareSample& sample);

    /*
     * Process one parsed Protocol-v1 command.
     *
     * MOVE is deliberately inhibited in this firmware phase.
     */
    [[nodiscard]]
    CommandDisposition handle_command(const HostCommand& command);

    [[nodiscard]]
    std::optional<MotorDriveCommand> drive_command() const;

    /*
     * Build the next telemetry packet.
     */
    [[nodiscard]]
    TelemetryPacket make_telemetry(bool driver_enabled = false);

  private:
    SafetySupervisor safety_;

    MotionAuthority motion_authority_;

    bool have_sample_ = false;
    bool baselines_valid_ = false;

    RawHardwareSample raw_{};

    std::int32_t motor_1_baseline_count_ = 0;
    std::int32_t motor_2_baseline_count_ = 0;

    std::int32_t motor_1_target_winding_counts_ = 0;
    std::int32_t motor_2_target_winding_counts_ = 0;

    std::uint32_t telemetry_sequence_ = 0;

    [[nodiscard]]
    SafetySample safety_sample() const;

    [[nodiscard]]
    std::int32_t motor_1_winding_counts() const;

    [[nodiscard]]
    std::int32_t motor_2_winding_counts() const;
};

/*
 * Preserve all existing dry-run tests/source compatibility.
 *
 * Default authority remains inhibited.
 */

} // namespace atj::h0
