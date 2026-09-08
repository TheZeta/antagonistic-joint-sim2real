#include "atj/controller.hpp"
#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>

int main() {
    constexpr std::array<double, 3> true_joint_radii{0.018, 0.020, 0.022};

    /*
     * This is the model believed by the controller.
     */
    const atj::Parameters nominal_parameters{.inertia = 0.01,
                                             .damping = 0.02,
                                             .spring_stiffness = 200.0,
                                             .joint_radius = 0.020,
                                             .motor_radius = 0.010,
                                             .preload = 0.010};

    constexpr double kp = 8.0;
    constexpr double kd = 3.0;

    const atj::PositionPDController controller(kp, kd);

    constexpr double q_reference = 20.0 * std::numbers::pi / 180.0;

    constexpr double common_mode = 1.5;

    const double theta_d_feedforward =
        atj::equilibrium_differential_command(nominal_parameters, q_reference);

    std::array<atj::JointModel, 3> plants{
        atj::JointModel(atj::Parameters{.inertia = nominal_parameters.inertia,
                                        .damping = nominal_parameters.damping,
                                        .spring_stiffness = nominal_parameters.spring_stiffness,
                                        .joint_radius = true_joint_radii[0],
                                        .motor_radius = nominal_parameters.motor_radius,
                                        .preload = nominal_parameters.preload}),

        atj::JointModel(atj::Parameters{.inertia = nominal_parameters.inertia,
                                        .damping = nominal_parameters.damping,
                                        .spring_stiffness = nominal_parameters.spring_stiffness,
                                        .joint_radius = true_joint_radii[1],
                                        .motor_radius = nominal_parameters.motor_radius,
                                        .preload = nominal_parameters.preload}),

        atj::JointModel(atj::Parameters{.inertia = nominal_parameters.inertia,
                                        .damping = nominal_parameters.damping,
                                        .spring_stiffness = nominal_parameters.spring_stiffness,
                                        .joint_radius = true_joint_radii[2],
                                        .motor_radius = nominal_parameters.motor_radius,
                                        .preload = nominal_parameters.preload})};

    std::array<atj::State, 3> states{atj::State{.q = 0.0, .q_dot = 0.0},
                                     atj::State{.q = 0.0, .q_dot = 0.0},
                                     atj::State{.q = 0.0, .q_dot = 0.0}};

    std::array<double, 3> minimum_extension_1{1.0, 1.0, 1.0};

    std::array<double, 3> minimum_extension_2{1.0, 1.0, 1.0};

    constexpr double dt = 0.01;
    constexpr double duration = 10.0;

    const int steps = static_cast<int>(duration / dt);

    std::filesystem::create_directories("results");

    std::ofstream output("results/model_mismatch_sweep.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";
        return 1;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_reference_rad,"
           << "q_rj_0p018_rad,"
           << "q_rj_0p020_rad,"
           << "q_rj_0p022_rad" << '\n';

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        output << time << ',' << q_reference << ',' << states[0].q << ',' << states[1].q << ','
               << states[2].q << '\n';

        for (std::size_t i = 0; i < states.size(); ++i) {
            const double theta_d_feedback = controller.differential_command(q_reference, states[i]);

            const double theta_d = theta_d_feedforward + theta_d_feedback;

            const atj::Input input = atj::make_antagonistic_input(theta_d, common_mode);

            const double extension_1 = plants[i].tendon_1_extension(states[i], input);
            const double extension_2 = plants[i].tendon_2_extension(states[i], input);

            minimum_extension_1[i] = std::min(minimum_extension_1[i], extension_1);
            minimum_extension_2[i] = std::min(minimum_extension_2[i], extension_2);

            if (step != steps) {
                states[i] = atj::rk4_step(plants[i], states[i], input, dt);
            }
        }
    }

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Model-mismatch sweep complete.\n\n"
              << "Controller-assumed r_j: " << nominal_parameters.joint_radius << " m\n"
              << "Feedforward theta_d:    " << theta_d_feedforward << " rad\n\n";

    for (std::size_t i = 0; i < states.size(); ++i) {
        const double true_rj = true_joint_radii[i];

        const double true_kq = 2.0 * nominal_parameters.spring_stiffness * true_rj * true_rj;

        const double true_ku =
            nominal_parameters.spring_stiffness * true_rj * nominal_parameters.motor_radius;

        const double predicted_equilibrium =
            (true_ku * (theta_d_feedforward + kp * q_reference)) / (true_kq + true_ku * kp);

        const double final_error = q_reference - states[i].q;

        std::cout << "True r_j = " << true_rj << " m\n"
                  << "  predicted q: " << predicted_equilibrium << " rad\n"
                  << "  final q: " << states[i].q << " rad\n"
                  << "  final error: " << final_error << " rad\n"
                  << "  min delta_1: " << minimum_extension_1[i] << " m\n"
                  << "  min delta_2: " << minimum_extension_2[i] << " m\n\n";
    }

    return 0;
}
