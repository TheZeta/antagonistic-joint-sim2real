#include "atj/h0_calibration.hpp"
#include "atj/h0_experiment_schedule.hpp"
#include "atj/h0_model_v1.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numbers>
#include <span>
#include <string_view>

namespace {

constexpr double model_v0_joint_radius_m = 0.020;

double model_v0_quasistatic_deg(double differential_offset_counts) {
    /*
     * Physical motor-coordinate differential:
     *
     * w_d = w2 - w1
     *
     * For:
     *
     * m1 = preload - offset
     * m2 = preload + offset
     *
     * therefore:
     *
     * w_d = 2 * offset * meters_per_count
     *
     * Model v0 equilibrium:
     *
     * q = w_d / (2*r_j)
     *
     * hence:
     *
     * q = offset * meters_per_count / r_j.
     */
    const double q_rad =
        differential_offset_counts * atj::h0::meters_per_motor_count / model_v0_joint_radius_m;

    return q_rad * 180.0 / std::numbers::pi;
}

bool run_batch(std::span<const std::int32_t> offsets, const char* output_path) {
    std::ofstream output{output_path};

    if (!output) {
        return false;
    }

    output << "step,"
           << "offset_counts,"
           << "model_v0_q_deg,"
           << "model_v1_q_deg,"
           << "model_v1_equilibrium_deg,"
           << "model_v1_memory_state\n";

    atj::h0::ModelV1 model_v1;

    /*
     * Experimental normalization:
     *
     * initial preload center:
     * x = 0 counts
     * q = 0 deg
     */
    model_v1.reset_from_equilibrium(0.0, 0.0);

    std::size_t step = 0;

    /*
     * Record initial preload state.
     */
    output << step++ << ',' << 0 << ',' << 0.0 << ',' << model_v1.q_deg() << ','
           << model_v1.equilibrium_deg(0.0) << ',' << model_v1.memory_state() << '\n';

    for (const std::int32_t offset : offsets) {
        const double model_v0_q = model_v0_quasistatic_deg(static_cast<double>(offset));

        /*
         * Move instantaneously to the new commanded
         * differential coordinate, then allow the model
         * to evolve for the same 5 s dwell used by H0.
         */
        const double model_v1_q =
            model_v1.update_deg(atj::h0::quasistatic_dwell_s, static_cast<double>(offset));

        output << step++ << ',' << offset << ',' << model_v0_q << ',' << model_v1_q << ','
               << model_v1.equilibrium_deg(static_cast<double>(offset)) << ','
               << model_v1.memory_state() << '\n';
    }

    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <quasistatic|minor> " << "<output.csv>\n";

        return 1;
    }

    const std::string_view experiment{argv[1]};

    if (experiment == "quasistatic") {
        if (!run_batch(atj::h0::quasistatic_offsets, argv[2])) {
            std::cerr << "Failed to write output CSV\n";

            return 1;
        }
    } else if (experiment == "minor") {
        if (!run_batch(atj::h0::minor_loop_offsets, argv[2])) {
            std::cerr << "Failed to write output CSV\n";

            return 1;
        }
    } else {
        std::cerr << "Unknown experiment: " << experiment << '\n';

        return 1;
    }

    std::cout << "Model batch complete: " << argv[2] << '\n';

    return 0;
}
