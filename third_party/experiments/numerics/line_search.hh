#pragma once

#include "numerics/optimization_problem.hh"

#include "eigen.hh"

#include <limits>

namespace numerics {

struct LinesearchResult {
  bool valid = false;
  double cost = std::numeric_limits<double>::max();
  Eigen::VectorXd best_x;
};

LinesearchResult line_search(const OptimizationState& current_state,
                             const Eigen::VectorXd& direction,
                             const OptimizationProblem& problem);
}  // namespace numerics