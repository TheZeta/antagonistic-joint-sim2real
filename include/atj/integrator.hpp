#pragma once

#include "atj/joint_model.hpp"

namespace atj {

[[nodiscard]]
State euler_step(const JointModel& model, const State& state, const Input& input, double dt);

[[nodiscard]]
State rk4_step(const JointModel& model, const State& state, const Input& input, double dt);

} // namespace atj
