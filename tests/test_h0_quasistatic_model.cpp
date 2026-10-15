#include "atj/h0_quasistatic_model.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("H0 quasistatic model predicts positive branches") {
    const atj::h0::QuasistaticModel model;

    const double outward = model.predict_deg(10.0, atj::h0::DifferentialDirection::increasing);
    const double returning = model.predict_deg(10.0, atj::h0::DifferentialDirection::decreasing);

    REQUIRE(outward == Catch::Approx(1.607));
    REQUIRE(returning == Catch::Approx(3.363));
    REQUIRE(returning > outward);
}

TEST_CASE("H0 quasistatic model predicts negative branches") {
    const atj::h0::QuasistaticModel model;

    /*
     * Negative outward sweep means differential
     * coordinate is decreasing.
     */
    const double outward = model.predict_deg(-10.0, atj::h0::DifferentialDirection::decreasing);

    /*
     * Negative return means differential
     * coordinate is increasing.
     */
    const double returning = model.predict_deg(-10.0, atj::h0::DifferentialDirection::increasing);

    REQUIRE(outward == Catch::Approx(-2.201));
    REQUIRE(returning == Catch::Approx(-3.957));
    REQUIRE(returning < outward);
}

TEST_CASE("H0 direction memory creates fitted branch separation") {
    const atj::h0::QuasistaticModel model;

    const double increasing = model.predict_deg(15.0, atj::h0::DifferentialDirection::increasing);
    const double decreasing = model.predict_deg(15.0, atj::h0::DifferentialDirection::decreasing);

    REQUIRE(decreasing - increasing == Catch::Approx(1.756));
}
