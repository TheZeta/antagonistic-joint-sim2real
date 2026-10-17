#include "atj/h0_experiment_schedule.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdlib>

TEST_CASE("H0 quasistatic schedule reaches full safe range") {
    const auto& offsets = atj::h0::quasistatic_offsets;

    REQUIRE(*std::max_element(offsets.begin(), offsets.end()) == 30);
    REQUIRE(*std::min_element(offsets.begin(), offsets.end()) == -30);
    REQUIRE(offsets.back() == 0);
}

TEST_CASE("H0 quasistatic schedule changes at most five counts") {
    const auto& offsets = atj::h0::quasistatic_offsets;

    std::int32_t previous = 0;

    for (const std::int32_t offset : offsets) {
        REQUIRE(std::abs(offset - previous) <= 5);

        previous = offset;
    }
}

TEST_CASE("H0 minor-loop schedule contains partial reversals") {
    const auto& offsets = atj::h0::minor_loop_offsets;

    REQUIRE(std::find(offsets.begin(), offsets.end(), 18) != offsets.end());
    REQUIRE(std::find(offsets.begin(), offsets.end(), -18) != offsets.end());
}
