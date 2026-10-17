#include "atj/h0_model_v1.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 Model v1 memory saturates during increasing motion") {
    atj::h0::ModelV1 model;

    model.reset(0.0, 0.0, 0.0);
    model.update_deg(0.0, 5.0);

    REQUIRE(model.memory_state() == Catch::Approx(0.779).margin(0.002));

    model.update_deg(0.0, 10.0);

    REQUIRE(model.memory_state() == Catch::Approx(0.951).margin(0.002));
}

TEST_CASE("H0 Model v1 memory reverses continuously") {
    atj::h0::ModelV1 model;

    model.reset(30.0, 0.0, 1.0);
    model.update_deg(0.0, 25.0);

    REQUIRE(model.memory_state() == Catch::Approx(-0.558).margin(0.002));

    model.update_deg(0.0, 20.0);

    REQUIRE(model.memory_state() == Catch::Approx(-0.902).margin(0.002));
}

TEST_CASE("H0 Model v1 memory does not decay while holding") {
    atj::h0::ModelV1 model;

    model.reset(10.0, 0.0, 0.75);

    for (int i = 0; i < 1000; ++i) {
        model.update_deg(0.001, 10.0);
    }

    REQUIRE(model.memory_state() == Catch::Approx(0.75));
}

TEST_CASE("H0 Model v1 relaxes toward memory-dependent equilibrium") {
    atj::h0::ModelV1 model;

    model.reset(0.0, 0.0, 0.0);
    model.update_deg(0.0, 20.0);

    const double equilibrium = model.equilibrium_deg(20.0);
    const double tau = model.parameters().relaxation_time_s;
    const double q = model.update_deg(tau, 20.0);
    const double expected = equilibrium * (1.0 - std::exp(-1.0));

    REQUIRE(q == Catch::Approx(expected).margin(0.001));
}

TEST_CASE("H0 Model v1 can infer initial memory from equilibrium") {
    atj::h0::ModelV1 model;

    /*
     * Our normalized initial experimental state:
     *
     * x = 0 counts
     * q = 0 deg
     */
    model.reset_from_equilibrium(0.0, 0.0);

    REQUIRE(model.memory_state() == Catch::Approx(-0.297).margin(0.002));
    REQUIRE(model.equilibrium_deg(0.0) == Catch::Approx(0.0).margin(0.001));
}

TEST_CASE("H0 Movel v1 reproduces positive minor reversal state") {
    atj::h0::ModelV1 model;

    /*
     * Experimental normalization:
     *
     * x = 0
     * q = 0
     *
     * Infer the corresponding initial memory.
     */
    model.reset_from_equilibrium(0.0, 0.0);

    /*
     * Follow the exact positive minor-loop approach:
     *
     * 0 -> 5 -> 10 -> 15 -> 20 -> 18
     *
     * dt = 0 because this test concerns the
     * displacement-driven hysteresis state only.
     */
    model.update_deg(0.0, 5.0);
    model.update_deg(0.0, 10.0);
    model.update_deg(0.0, 15.0);
    model.update_deg(0.0, 20.0);
    model.update_deg(0.0, 18.0);

    REQUIRE(model.memory_state() == Catch::Approx(0.091).margin(0.003));
    REQUIRE(model.equilibrium_deg(18.0) == Catch::Approx(4.707).margin(0.01));
}

TEST_CASE("H0 Model v1 reproduces negative minor reversal state") {
    atj::h0::ModelV1 model;

    model.reset_from_equilibrium(0.0, 0.0);

    /*
     * First drive positive memory to saturation,
     * then traverse through zero to the negative side,
     * matching the experiment.
     */
    model.update_deg(0.0, 5.0);
    model.update_deg(0.0, 10.0);
    model.update_deg(0.0, 15.0);
    model.update_deg(0.0, 20.0);

    model.update_deg(0.0, 18.0);
    model.update_deg(0.0, 20.0);

    model.update_deg(0.0, 15.0);
    model.update_deg(0.0, 10.0);
    model.update_deg(0.0, 5.0);
    model.update_deg(0.0, 0.0);
    model.update_deg(0.0, -5.0);
    model.update_deg(0.0, -10.0);
    model.update_deg(0.0, -15.0);
    model.update_deg(0.0, -20.0);

    model.update_deg(0.0, -18.0);

    REQUIRE(model.memory_state() == Catch::Approx(-0.093).margin(0.003));
    REQUIRE(model.equilibrium_deg(-18.0) == Catch::Approx(-5.273).margin(0.01));
}
