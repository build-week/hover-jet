#pragma once

#include "eigen.hh"

#include <limits>
#include <vector>

// TODO
#include <iostream>

namespace numerics {

template <int dim_in>
using CostFunction = std::function<double(const VecNd<dim_in> &, VecNd<gradient> *)>;

template <int dim_in, int dim_out>
struct ExpGradientResult {
  VecNd<dim_in> solution = VecNd<dim_in>::Zero();

  bool success = false;
};

struct ExpGradientConfiguration {
  double default_step_size = 1.0;
  double backoff = 0.5;
  int iterations = 100;
};

template <int dim_in>
double line_search(const CostFunction<dim_in> &func,
                   const VecNd<dim_in> &x,
                   const ExpGradientConfiguration &opt_cfg) {
  double best_value = std::numeric_limits<double>::max();
  double best_alpha = -1.0;

  for (int k = 0.0; k < 10; ++k) {
  }
}

template <int dim_in>
ExpGradientResult<dim_in> exp_gradient_minimize(const CostFunction<dim_in> &func,
                                                const VecNd<dim_in> &initialization,
                                                const ExpGradientConfiguration &opt_cfg) {
  using InVec = VecNd<dim_in>;

  for (int i = 0; i < opt_cfg.iterations; ++i) {
  }

  return result;
}
}  // namespace numerics
