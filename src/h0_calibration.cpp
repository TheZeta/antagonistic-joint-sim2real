#include "atj/h0_calibration.hpp"

#include <numbers>
#include <stdexcept>

namespace atj::h0 {

int wrap_as5600_delta(int raw_delta) {
    int wrapped = raw_delta % as5600_counts_per_revolution;

    if (wrapped < -as5600_half_revolution_counts) {
        wrapped += as5600_counts_per_revolution;
    } else if (wrapped >= as5600_half_revolution_counts) {
        wrapped -= as5600_counts_per_revolution;
    }

    return wrapped;
}

double joint_angle_rad_from_raw(int qraw) {
    if (qraw < 0 || qraw >= as5600_counts_per_revolution) {
        throw std::out_of_range("AS5600 raw value must be in [0, 4095]");
    }

    const int raw_delta = qraw - joint_zero_raw;

    const int wrapped_delta = wrap_as5600_delta(raw_delta);

    constexpr double radians_per_count =
        2.0 * std::numbers::pi / static_cast<double>(as5600_counts_per_revolution);

    return static_cast<double>(wrapped_delta) * radians_per_count;
}

double winding_displacement_m_from_count_delta(int count_delta) {
    /*
     * WIND -> count decreases
     *
     * We deliberately expose the opposite convention
     * to the rest of the software:
     *
     * positive winding displacement -> WIND.
     */
    return -static_cast<double>(count_delta) * meters_per_motor_count;
}

double motor_count_delta_from_winding_displacement_m(double winding_displacement_m) {
    return -winding_displacement_m * motor_counts_per_meter;
}

} // namespace atj::h0
