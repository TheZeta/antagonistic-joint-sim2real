#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    constexpr std::array<double, 3> dampings{0.01, 0.02, 0.08};

    constexpr double inertia = 0.01;
    constexpr double spring_stiffness = 200.0;
    constexpr double joint_radius = 0.02;
    constexpr double motor_radius = 0.01;
    constexpr double preload = 0.01;

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    std::array<atj::JointModel, 3> models{
        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = dampings[0],
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preload}),

        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = dampings[1],
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preload}),

        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = dampings[2],
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preload})};

    std::array<atj::State, 3> states{
        atj::State{.q = 0.0, .q_dot = 0.0},
        atj::State{.q = 0.0, .q_dot = 0.0},
        atj::State{.q = 0.0, .q_dot = 0.0},
    };

    std::array<double, 3> peak_q{0.0, 0.0, 0.0};

    std::array<double, 3> peak_time{0.0, 0.0, 0.0};

    constexpr double dt = 0.01;
    constexpr double duration = 10.0;

    const int steps = static_cast<int>(duration / dt);

    std::filesystem::create_directories("results");

    std::ofstream output("results/damping_sweep.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";
        return 1;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_b_0p01_rad,"
           << "q_b_0p02_rad,"
           << "q_b_0p08_rad" << '\n';

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        output << time << ',' << states[0].q << ',' << states[1].q << ',' << states[2].q << '\n';

        for (std::size_t i = 0; i < states.size(); ++i) {
            if (states[i].q > peak_q[i]) {
                peak_q[i] = states[i].q;
                peak_time[i] = time;
            }
        }

        if (step == steps) {
            break;
        }

        for (std::size_t i = 0; i < states.size(); ++i) {
            states[i] = atj::rk4_step(models[i], states[i], input, dt);
        }
    }

    std::cout << "Damping sweep complete.\n\n";

    for (std::size_t i = 0; i < dampings.size(); ++i) {
        std::cout << "b = " << dampings[i] << " N m s/rad\n"
                  << "  peak q:    " << peak_q[i] << " rad\n"
                  << "  peak time: " << peak_time[i] << " s\n"
                  << "  final q:   " << states[i].q << " rad\n\n";
    }

    return 0;
}
