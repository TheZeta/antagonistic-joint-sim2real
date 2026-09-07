#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    constexpr std::array<double, 3> preloads{0.005, 0.010, 0.020};

    constexpr double inertia = 0.01;
    constexpr double damping = 0.02;
    constexpr double spring_stiffness = 200.0;
    constexpr double joint_radius = 0.02;
    constexpr double motor_radius = 0.01;

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    std::array<atj::JointModel, 3> models{
        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = damping,
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preloads[0]}),

        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = damping,
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preloads[1]}),

        atj::JointModel(atj::Parameters{.inertia = inertia,
                                        .damping = damping,
                                        .spring_stiffness = spring_stiffness,
                                        .joint_radius = joint_radius,
                                        .motor_radius = motor_radius,
                                        .preload = preloads[2]})};

    std::array<atj::State, 3> states{atj::State{.q = 0.0, .q_dot = 0.0},
                                     atj::State{.q = 0.0, .q_dot = 0.0},
                                     atj::State{.q = 0.0, .q_dot = 0.0}};

    constexpr double dt = 0.01;
    constexpr double duration = 6.0;

    const int steps = static_cast<int>(duration / dt);

    std::filesystem::create_directories("results");

    std::ofstream output("results/preload_sweep.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";
        return 1;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_preload_0p005_rad,"
           << "q_preload_0p010_rad,"
           << "q_preload_0p020_rad,"
           << "f1_preload_0p005_N,"
           << "f2_preload_0p005_N,"
           << "f1_preload_0p010_N,"
           << "f2_preload_0p010_N,"
           << "f1_preload_0p020_N,"
           << "f2_preload_0p020_N" << '\n';

    double max_position_difference = 0.0;

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        const double difference_low_high = std::abs(states[0].q - states[2].q);

        if (difference_low_high > max_position_difference) {
            max_position_difference = difference_low_high;
        }

        output << time << ',' << states[0].q << ',' << states[1].q << ',' << states[2].q << ','
               << models[0].tendon_1_force(states[0], input) << ','
               << models[0].tendon_2_force(states[0], input) << ','
               << models[1].tendon_1_force(states[1], input) << ','
               << models[1].tendon_2_force(states[1], input) << ','
               << models[2].tendon_1_force(states[2], input) << ','
               << models[2].tendon_2_force(states[2], input) << '\n';

        if (step == steps) {
            break;
        }

        for (std::size_t i = 0; i < states.size(); ++i) {
            states[i] = atj::rk4_step(models[i], states[i], input, dt);
        }
    }

    std::cout << "Preload sweep complete.\n\n"
              << "Maximum position difference between\n"
              << "lowest and highest preload: " << max_position_difference << " rad\n\n";

    for (std::size_t i = 0; i < preloads.size(); ++i) {
        std::cout << "preload = " << preloads[i] << " m\n"
                  << "  final q:  " << states[i].q << " rad\n"
                  << "  final F1: " << models[i].tendon_1_force(states[i], input) << " N\n"
                  << "  final F2: " << models[i].tendon_2_force(states[i], input) << " N\n\n";
    }

    return 0;
}
