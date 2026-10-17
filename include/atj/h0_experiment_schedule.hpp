#pragma once

#include <array>
#include <cstdint>

namespace atj::h0 {

inline constexpr std::array<std::int32_t, 24> quasistatic_offsets{
    5,  10,  15,  20,  25,  30,  25,  20,  15,  10,  5,  0,
    -5, -10, -15, -20, -25, -30, -25, -20, -15, -10, -5, 0};

inline constexpr std::array<std::int32_t, 20> minor_loop_offsets{
    5, 10, 15, 20, 18, 20, 15, 10, 5, 0, -5, -10, -15, -20, -18, -20, -15, -10, -5, 0};

inline constexpr double quasistatic_dwell_s = 5.0;

} // namespace atj::h0
