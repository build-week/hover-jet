#pragma once

#include "estimation/jet/jet_rk4.hh"
#include "estimation/optimization/acausal_optimizer.hh"

namespace estimation {
namespace jet_filter {

using JetOptimizerProblem = optimization::ProblemType<State, Parameters>;
using JetPoseOptimizer = optimization::AcausalOptimizer<JetOptimizerProblem>;

}  // namespace jet_filter
}  // namespace estimation