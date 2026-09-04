#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    const atj::Parameters parameters{.inertia = 0.01,
                                     .damping = 0.02,
                                     .spring_stiffness = 200.0,
                                     .joint_radius = 0.02,
                                     .motor_radius = 0.01,
                                     .preload = 0.01};

    const atj::JointModel model(parameters);

    atj::State euler_state{.q = 0.0, .q_dot = 0.0};

    atj::State rk4_state{.q = 0.0, .q_dot = 0.0};

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    constexpr double dt = 0.01;
    constexpr double duration = 6.0;

    const int steps = static_cast<int>(duration / dt);

    std::filesystem::create_directories("results");

    std::ofstream output("results/step_response.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";

        return 1;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_euler_rad,"
           << "qdot_euler_rad_s,"
           << "q_rk4_rad,"
           << "qdot_rk4_rad_s,"
           << "tendon_1_extension_m,"
           << "tendon_2_extension_m,"
           << "tendon_1_force_N,"
           << "tendon_2_force_N" << '\n';

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        const double extension_1 = model.tendon_1_extension(rk4_state, input);
        const double extension_2 = model.tendon_2_extension(rk4_state, input);
        const double force_1 = model.tendon_1_force(rk4_state, input);
        const double force_2 = model.tendon_2_force(rk4_state, input);

        output << time << ',' << euler_state.q << ',' << euler_state.q_dot << ',' << rk4_state.q
               << ',' << rk4_state.q_dot << ',' << extension_1 << ',' << extension_2 << ','
               << force_1 << ',' << force_2 << '\n';

        if (step == steps) {
            break;
        }

        euler_state = atj::euler_step(model, euler_state, input, dt);
        rk4_state = atj::rk4_step(model, rk4_state, input, dt);
    }

    std::cout << "Simulation complete.\n"
              << "Results written to "
              << "results/step_response_csv\n"
              << '\n'
              << "Final RK4 state:\n"
              << "  q     = " << rk4_state.q << " rad\n"
              << "  q_dot = " << rk4_state.q_dot << " rad/s\n";

    return 0;
}
