#include "atj/h0_telemetry.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 telemetry converts raw packet to physical coordinates") {
    const atj::h0::TelemetryPacket packet{.sequence = 42,
                                          .time_us = 1'500'000,

                                          .qraw = 0,

                                          .motor_1_raw_count = 1201,
                                          .motor_2_raw_count = 1799,

                                          .motor_1_baseline_count = 1000,
                                          .motor_2_baseline_count = 2000,

                                          .motor_1_winding_counts = -201,
                                          .motor_2_winding_counts = 201,

                                          .motor_1_target_winding_counts = -201,
                                          .motor_2_target_winding_counts = 201,

                                          .state = atj::h0::SafetyState::holding,
                                          .fault = atj::h0::SafetyFault::none,

                                          .driver_enabled = false};

    const auto physical = atj::h0::physical_telemetry(packet);

    REQUIRE(physical.time_s == Catch::Approx(1.5));

    /*
     * QRAW0 = 3840.
     *
     * Raw zero is +256 counts across rollover,
     * corresponding to +22.5 deg.
     */
    REQUIRE(physical.q_deg == Catch::Approx(22.5));
    REQUIRE(physical.motor_1_winding_m == Catch::Approx(-0.023));
    REQUIRE(physical.motor_2_winding_m == Catch::Approx(0.023));
    REQUIRE(physical.common_winding_m == Catch::Approx(0.0));
    REQUIRE(physical.differential_winding_m == Catch::Approx(0.046));
}
