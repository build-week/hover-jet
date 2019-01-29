#pragma once

#include "eigen.hh"

#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {

struct StateControl {
  State state;
  Controls control;
};

struct Desires {
  jcc::Vec3 target;

  // TEMPORARY
  double supp_v_weight = 0.0;
};

std::vector<StateControl> plan(const State& x0,
                               const Desires& desires,
                               const std::vector<Controls>& initialization = {});

Parameters get_parameters();

}  // namespace jet
}  // namespace planning