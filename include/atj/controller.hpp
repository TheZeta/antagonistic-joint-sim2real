#pragma once

#include "atj/joint_model.hpp"

namespace atj {

class PositionPController {
  public:
    explicit PositionPController(double kp);

    [[nodiscard]]
    double differential_command(double q_reference, const State& state) const;

  private:
    double kp_;
};

[[nodiscard]]
Input make_antagonistic_input(double differential_command, double common_mode);

class PositionPDController {
  public:
    PositionPDController(double kp, double kd);

    [[nodiscard]]
    double differential_command(double q_reference, const State& state) const;

  private:
    double kp_;
    double kd_;
};
} // namespace atj
