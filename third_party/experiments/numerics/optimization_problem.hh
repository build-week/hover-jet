#pragma once

#include "numerics/cost_function.hh"

#include "eigen.hh"

namespace numerics {

namespace detail {
using Vecx = Eigen::VectorXd;
using Matx = Eigen::MatrixXd;
}  // namespace detail

struct BoxConstraint {
  // If max/min are equal to double precision, this will be treated
  // as an equality constraint, which is a bit more stable than two inequality
  // constraints
  int dimension = -1;
  double max = 0.0;
  double min = 0.0;
};

struct Constraint {
  CostFunction g;
  bool mirrored = false;
};

struct OptimizationProblem {
  CostFunction objective;
  std::vector<Constraint> func_constraints;
  std::vector<BoxConstraint> box_constraints;
};

struct OptimizationState {
  detail::Vecx x;
  detail::Vecx lambda;
};

}  // namespace numerics