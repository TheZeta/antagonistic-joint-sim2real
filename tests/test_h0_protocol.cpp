#include "atj/h0_protocol.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 normalized winding counts hide raw encoder polarity") {
    constexpr std::int32_t baseline = 1000;

    SECTION("WIND") {
        /*
         * WIND caused raw encoder count to decrease.
         */
        constexpr std::int32_t raw = 799;

        REQUIRE(atj::h0::winding_count_from_raw(raw, baseline) == 201);
    }

    SECTION("UNWIND") {
        constexpr std::int32_t raw = 1201;

        REQUIRE(atj::h0::winding_count_from_raw(raw, baseline) == -201);
    }
}

TEST_CASE("H0 normalized winding target converts to raw target") {
    constexpr std::int32_t baseline = 1000;

    REQUIRE(atj::h0::raw_target_from_winding_count(baseline, 201) == 799);
    REQUIRE(atj::h0::raw_target_from_winding_count(baseline, -201) == 1201);
}

TEST_CASE("H0 q-positive differential command reproduces commissioned motor directions") {
    constexpr std::int32_t baseline_1 = 1000;
    constexpr std::int32_t baseline_2 = 2000;

    /*
     * q+ commissioning result:
     *
     * Motor 1 UNWIND
     * Motor 2 WIND
     *
     * Therefore normalized winding:
     *
     * W1 < 0
     * W2 > 0
     */
    constexpr std::int32_t target_1 = -201;
    constexpr std::int32_t target_2 = 201;

    const auto raw_target_1 = atj::h0::raw_target_from_winding_count(baseline_1, target_1);
    const auto raw_target_2 = atj::h0::raw_target_from_winding_count(baseline_2, target_2);

    REQUIRE(raw_target_1 == 1201);
    REQUIRE(raw_target_2 == 1799);

    /*
     * Raw map:
     *
     * Motor 1 count increased -> UNWIND
     * Motor 2 count decreased -> WIND
     *
     * Exactly the commissioned q+ actuation map.
     */
}

TEST_CASE("H0 MOVE command round-trips") {
    const atj::h0::HostCommand command{.sequence = 42,
                                       .type = atj::h0::CommandType::move,
                                       .motor_1_target_winding_counts = -120,
                                       .motor_2_target_winding_counts = 120};

    const std::string serialized = atj::h0::serialize_command(command);

    REQUIRE(serialized == "H0C,1,42,MOVE,-120,120\n");

    const auto parsed = atj::h0::parse_command(serialized);

    REQUIRE(parsed.has_value());
    REQUIRE(parsed->sequence == 42);
    REQUIRE(parsed->type == atj::h0::CommandType::move);
    REQUIRE(parsed->motor_1_target_winding_counts == -120);
    REQUIRE(parsed->motor_2_target_winding_counts == 120);
}

TEST_CASE("H0 ARM command round-trips") {
    const atj::h0::HostCommand command{.sequence = 7, .type = atj::h0::CommandType::arm};

    const auto parsed = atj::h0::parse_command(atj::h0::serialize_command(command));

    REQUIRE(parsed.has_value());
    REQUIRE(parsed->type == atj::h0::CommandType::arm);
}

TEST_CASE("H0 protocol rejects malformed commands") {
    REQUIRE_FALSE(atj::h0::parse_command("garbage\n").has_value());
    REQUIRE_FALSE(atj::h0::parse_command("H0C,99,1,ARM\n").has_value());
    REQUIRE_FALSE(atj::h0::parse_command("H0C,1,1,MOVE,abc,12\n").has_value());
    REQUIRE_FALSE(atj::h0::parse_command("H0C,1,1,MOVE,12\n").has_value());
    REQUIRE_FALSE(atj::h0::parse_command("H0C,1,1,ENABLE\n").has_value());
}

TEST_CASE("H0 telemetry serialization has expected wire format") {
    const atj::h0::TelemetryPacket packet{.sequence = 81,
                                          .time_us = 1523400,

                                          .qraw = 3842,

                                          .motor_1_raw_count = -1045,
                                          .motor_2_raw_count = 934,

                                          .motor_1_baseline_count = -1000,
                                          .motor_2_baseline_count = 1000,

                                          .motor_1_winding_counts = 45,
                                          .motor_2_winding_counts = 66,

                                          .motor_1_target_winding_counts = 50,
                                          .motor_2_target_winding_counts = 70,

                                          .state = atj::h0::SafetyState::moving,
                                          .fault = atj::h0::SafetyFault::none,

                                          .driver_enabled = true};

    REQUIRE(atj::h0::serialize_telemetry(packet) ==
            "H0T,1,81,1523400,3842,-1045,934,-1000,1000,45,66,50,70,2,0,1\n");
}

TEST_CASE("H0 telemetry parser accepts valid wire packet") {
    constexpr auto line = "H0T,1,81,1523400,3842,-1045,934,-1000,1000,45,66,50,70,2,0,1\n";

    const auto parsed = atj::h0::parse_telemetry(line);

    REQUIRE(parsed.has_value());

    REQUIRE(parsed->sequence == 81);
    REQUIRE(parsed->time_us == 1523400);
    REQUIRE(parsed->qraw == 3842);

    REQUIRE(parsed->state == atj::h0::SafetyState::moving);
    REQUIRE(parsed->fault == atj::h0::SafetyFault::none);
    REQUIRE(parsed->driver_enabled);
}

TEST_CASE("H0 telemetry packet round-trips") {
    const atj::h0::TelemetryPacket packet{.sequence = 81,
                                          .time_us = 1523400,

                                          .qraw = 3842,

                                          .motor_1_raw_count = -1045,
                                          .motor_2_raw_count = 934,

                                          .motor_1_baseline_count = -1000,
                                          .motor_2_baseline_count = 1000,

                                          .motor_1_winding_counts = 45,
                                          .motor_2_winding_counts = 66,

                                          .motor_1_target_winding_counts = 50,
                                          .motor_2_target_winding_counts = 70,

                                          .state = atj::h0::SafetyState::moving,
                                          .fault = atj::h0::SafetyFault::none,

                                          .driver_enabled = true};

    const auto parsed = atj::h0::parse_telemetry(atj::h0::serialize_telemetry(packet));

    REQUIRE(parsed.has_value());
    REQUIRE(parsed->sequence == packet.sequence);
    REQUIRE(parsed->time_us == packet.time_us);
    REQUIRE(parsed->qraw == packet.qraw);
    REQUIRE(parsed->motor_1_raw_count == packet.motor_1_raw_count);
    REQUIRE(parsed->motor_2_raw_count == packet.motor_2_raw_count);
    REQUIRE(parsed->state == atj::h0::SafetyState::moving);
    REQUIRE(parsed->fault == atj::h0::SafetyFault::none);
    REQUIRE(parsed->driver_enabled);
}
