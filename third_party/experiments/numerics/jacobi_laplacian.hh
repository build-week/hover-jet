#pragma once

#include "eigen.hh"

// TODO
#include <iostream>

namespace numerics {

struct JacobiConfig {
  double alpha = -1e-1;
  double beta  = 4.0;

  int max_iters = -1;
};

template <int ROWS>
SquareMatNd<ROWS> jacobi_laplacian_image(const SquareMatNd<ROWS>& x,
                                         const SquareMatNd<ROWS>& b,
                                         const JacobiConfig&      config) {
  using Out           = SquareMatNd<ROWS>;
  const int max_iters = config.max_iters == -1 ? 2 * ROWS * ROWS : config.max_iters;

  Out x_soln = x;

  const double inv_beta = 1.0 / config.beta;

  for (int k = 0; k < max_iters; ++k) {
    Out result = Out::Zero();
    // x_{i - 1, j}
    result.template rightCols<ROWS - 1>() += x_soln.template leftCols<ROWS - 1>();

    // x_{i + 1, j}
    result.template leftCols<ROWS - 1>() += x_soln.template rightCols<ROWS - 1>();

    // x_{i, j - 1}
    result.template bottomRows<ROWS - 1>() += x_soln.template topRows<ROWS - 1>();

    // x_{i, j + 1}
    result.template topRows<ROWS - 1>() += x_soln.template bottomRows<ROWS - 1>();

    result += config.alpha * b;

    x_soln = result * inv_beta;
  }

  return x_soln;
}
}  // namespace numerics