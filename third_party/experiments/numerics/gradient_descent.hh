#pragma once

#include "eigen.hh"

namespace numerics {

struct GradientDescentConfig {
  int    max_iters              = -1;
  double cost_convergence_ratio = 1e-9;
};

using SimpleCostFunction = std::function<double(const Eigen::VectorXd &, Eigen::VectorXd *)>;

Eigen::VectorXd minimize_gradient_descent(const Eigen::VectorXd &      initialization,
                                          const SimpleCostFunction &   cost_func,
                                          const GradientDescentConfig &cfg = {});
}  // namespace numerics
