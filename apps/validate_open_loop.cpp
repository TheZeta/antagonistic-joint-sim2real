#include "atj/integrator.hpp"
#include "atj/joint_model.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

atj::State analytical_step_response(const atj::Parameters& parameters, const atj::Input& input,
                                    double time) {
    const double rotational_stiffness =
        2.0 * parameters.spring_stiffness * parameters.joint_radius * parameters.joint_radius;

    const double input_gain =
        parameters.spring_stiffness * parameters.joint_radius * parameters.motor_radius;

    const double theta_d = input.theta_1 - input.theta_2;

    const double q_equilibrium = (input_gain * theta_d) / rotational_stiffness;

    const double omega_n = std::sqrt(rotational_stiffness / parameters.inertia);

    const double alpha = parameters.damping / (2.0 * parameters.inertia);

    if (alpha >= omega_n) {
        std::cerr << "Analytical solution currently assumes an underdamped system.\n";
        std::exit(EXIT_FAILURE);
    }

    const double omega_d = std::sqrt(omega_n * omega_n - alpha * alpha);

    const double decay = std::exp(-alpha * time);

    const double q =
        q_equilibrium *
        (1.0 - decay * (std::cos(omega_d * time) + (alpha / omega_d) * std::sin(omega_d * time)));

    const double q_dot =
        q_equilibrium * (omega_n * omega_n / omega_d) * decay * std::sin(omega_d * time);

    return atj::State{.q = q, .q_dot = q_dot};
}

} // namespace

int main() {
    const atj::Parameters parameters{.inertia = 0.01,
                                     .damping = 0.02,
                                     .spring_stiffness = 200.0,
                                     .joint_radius = 0.02,
                                     .motor_radius = 0.01,
                                     .preload = 0.01};

    const atj::JointModel model(parameters);

    const atj::Input input{.theta_1 = 1.0, .theta_2 = 0.0};

    atj::State numerical_state{.q = 0.0, .q_dot = 0.0};

    constexpr double dt = 0.01;
    constexpr double duration = 6.0;

    const int steps = static_cast<int>(duration / dt);

    std::filesystem::create_directories("results");

    std::ofstream output("results/open_loop_validation.csv");

    if (!output) {
        std::cerr << "Failed to open output CSV.\n";
        return EXIT_FAILURE;
    }

    output << std::setprecision(15);

    output << "time_s,"
           << "q_rk4_rad,"
           << "q_analytical_rad,"
           << "q_error_rad,"
           << "qdot_rk4_rad_s,"
           << "qdot_analytical_rad_s,"
           << "qdot_error_rad_s" << '\n';

    double max_q_error = 0.0;
    double max_qdot_error = 0.0;
    double squared_q_error_sum = 0.0;

    int sample_count = 0;

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;

        const atj::State analytical_state = analytical_step_response(parameters, input, time);

        const double q_error = numerical_state.q - analytical_state.q;

        const double qdot_error = numerical_state.q_dot - analytical_state.q_dot;

        max_q_error = std::max(max_q_error, std::abs(q_error));

        max_qdot_error = std::max(max_qdot_error, std::abs(qdot_error));

        squared_q_error_sum += q_error * q_error;

        ++sample_count;

        output << time << ',' << numerical_state.q << ',' << analytical_state.q << ',' << q_error
               << ',' << numerical_state.q_dot << ',' << analytical_state.q_dot << ',' << qdot_error
               << '\n';

        if (step == steps) {
            break;
        }

        numerical_state = atj::rk4_step(model, numerical_state, input, dt);
    }

    const double q_rmse = std::sqrt(squared_q_error_sum / static_cast<double>(sample_count));

    std::cout << std::scientific;

    std::cout << "Open-loop validation complete.\n\n"
              << "Maximum |q error|:      " << max_q_error << " rad\n"
              << "Maximum |q_dot error|:  " << max_qdot_error << " rad/s\n"
              << "q RMSE:                 " << q_rmse << " rad\n";

    return EXIT_SUCCESS;
}
