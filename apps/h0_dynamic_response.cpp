#include "atj/h0_dynamic_model.hpp"

#include <fstream>
#include <iostream>

int main() {
    constexpr double dt_s = 0.001;
    constexpr double duration_s = 5.0;

    atj::h0::DynamicQuasistaticModel model;

    /*
     * Start at the identified zero-offset equilibrium while
     * differential coordinate is increasing.
     */
    const atj::h0::QuasistaticModel static_model;

    model.reset_deg(static_model.predict_deg(0.0, atj::h0::DifferentialDirection::increasing));
    std::ofstream output{"h0_dynamic_response.csv"};

    if (!output) {
        std::cerr << "Failed to open output CSV\n";

        return 1;
    }

    output << "time_s,"
           << "offset_counts,"
           << "q_deg,"
           << "q_equilibrium_deg\n";

    for (double time_s = 0.0; time_s <= duration_s; time_s += dt_s) {
        const double offset = time_s < 0.5 ? 0.0 : 20.0;
        const auto direction = atj::h0::DifferentialDirection::increasing;
        const double q = model.update_deg(dt_s, offset, direction);
        const double equilibrium = static_model.predict_deg(offset, direction);

        output << time_s << ',' << offset << ',' << q << ',' << equilibrium << '\n';
    }

    std::cout << "Wrote h0_dynamic_response.csv\n";

    return 0;
}
