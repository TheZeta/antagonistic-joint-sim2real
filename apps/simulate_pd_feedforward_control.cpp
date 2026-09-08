#include "atj/controller.hpp"
#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>

int main() {
    const atj::Parameters parameters{.inertia = 0.01,
                                     .damping = 0.02,
                                     .spring_stiffness = 200.0,
                                     .joint_radius = 0.02,
                                     .motor_radius = 0.01,
                                     .preload = 0.01};

    const atj::JointModel model(parameters);

    constexpr double kp = 8.0;
    constexpr double kd = 3.0;

    const atj::PositionPDController controller(kp, kd);

    constexpr double q_reference = 20.0 * std::numbers::pi / 180.0;

    /*
     * Common-mode actuation keeps both tendons comfortably
     * tensioned while the differential controller moves the joint.
     *
     * In Model v0 this does not affect joint dynamics.
     */
    constexpr double common_mode = 1.5;

    atj::State state{.q = 0.0, .q_dot = 0.0};

    constexpr double dt = 0.01;
    constexpr double duration = 10.0;

    const int steps = static_cast<int>(duration / dt);

    const double theta_d_feedforward =
        atj::equilibrium_differential_command(parameters, q_reference);

    std::filesystem::create_directories("results");

    std::ofstream output("results/pd_feedforward_response.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";
        return 1;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_reference_rad,"
           << "q_rad,"
           << "qdot_rad_s,"
           << "error_rad,"
           << "theta_d_feedforward_rad,"
           << "theta_d_feedback_rad,"
           << "theta_d_rad,"
           << "theta_1_rad,"
           << "theta_2_rad,"
           << "tendon_1_force_N,"
           << "tendon_2_force_N" << '\n';

    double peak_q = state.q;

    double minimum_extension_1 =
        model.tendon_1_extension(state, atj::make_antagonistic_input(0.0, common_mode));

    double minimum_extension_2 =
        model.tendon_2_extension(state, atj::make_antagonistic_input(0.0, common_mode));

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        const double error = q_reference - state.q;

        const double theta_d_feedback = controller.differential_command(q_reference, state);

        const double theta_d = theta_d_feedforward + theta_d_feedback;

        const atj::Input input = atj::make_antagonistic_input(theta_d, common_mode);

        const double extension_1 = model.tendon_1_extension(state, input);
        const double extension_2 = model.tendon_2_extension(state, input);

        minimum_extension_1 = std::min(minimum_extension_1, extension_1);
        minimum_extension_2 = std::min(minimum_extension_2, extension_2);

        if (state.q > peak_q) {
            peak_q = state.q;
        }

        output << time << "," << q_reference << "," << state.q << "," << state.q_dot << "," << error
               << "," << theta_d_feedforward << "," << theta_d_feedback << "," << theta_d << ","
               << input.theta_1 << "," << input.theta_2 << "," << model.tendon_1_force(state, input)
               << "," << model.tendon_2_force(state, input) << "," << '\n';

        if (step == steps) {
            break;
        }

        /*
         * The controller is evaluated once every dt seconds.
         *
         * The resulting motor command is held constant over the
         * following integration interval. This corresponds to a
         * sampled digital controller with zero-order hold.
         */
        state = atj::rk4_step(model, state, input, dt);
    }

    const double final_error = q_reference - state.q;

    const double overshoot = std::max(0.0, peak_q - q_reference);

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "PD-feedforward-control simulation complete.\n\n"
              << "Reference q:                " << q_reference << " rad\n"
              << "Feedforward theta_d:        " << theta_d_feedforward << " rad\n"
              << "Final simulated q:          " << state.q << " rad\n"
              << "Final position error:       " << final_error << " rad\n"
              << "Maximum q:                  " << peak_q << " rad\n"
              << "Maximum overshoot:          " << overshoot << " rad\n"
              << "Minimum tendon 1 extension: " << minimum_extension_1 << " m\n"
              << "Minimum tendon 2 extension: " << minimum_extension_2 << " m\n";

    return 0;
}
