#pragma once

namespace atj::h0 {

/*
 * AS5600 absolute joint encoder
 */
inline constexpr int as5600_counts_per_revolution = 4096;
inline constexpr int as5600_half_revolution_counts = 2048;

inline constexpr int joint_zero_raw = 3840;

/*
 * Commissioned motor/tendon calibration:
 *
 *      201 encoder counts <-> 23 mm tendon travel
 *
 * WIND     -> raw motor encoder count decreases
 * UNWIND   -> raw motor encoder count increases
 *
 * We define positive winding displacement as WIND.
 */
inline constexpr double tendon_calibration_counts = 201.0;
inline constexpr double tendon_calibration_length_m = 0.023;

inline constexpr double meters_per_motor_count =
    tendon_calibration_length_m / tendon_calibration_counts;

inline constexpr double motor_counts_per_meter =
    tendon_calibration_counts / tendon_calibration_length_m;

/*
 * Wrap an AS5600 raw-count difference into:
 *
 *      [-2048, 2047]
 *
 * corresponding to:
 *
 *      [-pi, pi)
 */
[[nodiscard]]
int wrap_as5600_delta(int raw_delta);

/*
 * Convert AS5600 raw position to signed joint angle.
 *
 * Commissioned convention:
 *
 *      QRAW increasing -> q > 0
 *      QRAW decreasing -> q < 0
 *
 * Throws std::out_of_range if qraw is outside [0, 4095].
 */
[[nodiscard]]
double joint_angle_rad_from_raw(int qraw);

/*
 * Convert motor encoder-count change into physical tendon
 * winding displacement
 *
 * Positive result -> WIND
 * Negative result -> UNWIND
 */
[[nodiscard]]
double winding_displacement_m_from_count_delta(int count_delta);

/*
 * Inverse of winding_displacement_m_from_count_delta().
 *
 * Result is intentionally double; later hardware-command code
 * will explicitly decide how encoder-count targets are rounded.
 */
[[nodiscard]]
double motor_count_delta_from_winding_displacement_m(double winding_displacement_m);

} // namespace atj::h0
