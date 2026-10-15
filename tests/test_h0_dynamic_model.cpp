#include "atj/h0_dynamic_model.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

TEST_CASE("H0 dynamic model initializes at branch equilibrium") {
    atj::h0::DynamicQuasistaticModel model;

    const double q = model.update_deg(0.01, 10.0, atj::h0::DifferentialDirection::increasing);
    const atj::h0::QuasistaticModel equilibrium;

    REQUIRE(q == Catch::Approx(
                     equilibrium.predict_deg(10.0, atj::h0::DifferentialDirection::increasing)));
}

TEST_CASE("H0 dynamic model advances one time constant exactly") {
    constexpr double tau = 0.584;

    atj::h0::DynamicQuasistaticModel model{
        atj::h0::QuasistaticModel{}, atj::h0::DynamicModelParameters{.relaxation_time_s = tau}};

    model.reset_deg(0.0);

    const atj::h0::QuasistaticModel static_model;

    const double equilibrium =
        static_model.predict_deg(20.0, atj::h0::DifferentialDirection::increasing);
    const double q = model.update_deg(tau, 20.0, atj::h0::DifferentialDirection::increasing);
    const double expected = equilibrium * (1.0 - std::exp(-1.0));

    REQUIRE(q == Catch::Approx(expected).epsilon(1e-12));
}

TEST_CASE("H0 dynamic model converges to quasistatic branch") {
    atj::h0::DynamicQuasistaticModel model;

    model.reset_deg(0.0);

    const atj::h0::QuasistaticModel static_model;

    const double equilibrium =
        static_model.predict_deg(15.0, atj::h0::DifferentialDirection::increasing);

    double q = 0.0;

    for (int i = 0; i < 5000; ++i) {
        q = model.update_deg(0.001, 15.0, atj::h0::DifferentialDirection::increasing);
    }

    REQUIRE(q == Catch::Approx(equilibrium).margin(0.001));
}

TEST_CASE("H0 dynamic model retains direction-dependent equilibrium") {
    atj::h0::DynamicQuasistaticModel increasing;
    atj::h0::DynamicQuasistaticModel decreasing;

    increasing.reset_deg(0.0);
    decreasing.reset_deg(0.0);

    double q_increasing = 0.0;
    double q_decreasing = 0.0;

    for (int i = 0; i < 5000; ++i) {
        q_increasing =
            increasing.update_deg(0.001, 10.0, atj::h0::DifferentialDirection::increasing);
        q_decreasing =
            decreasing.update_deg(0.001, 10.0, atj::h0::DifferentialDirection::decreasing);
    }

    REQUIRE(q_decreasing - q_increasing == Catch::Approx(1.756).margin(0.001));
}
