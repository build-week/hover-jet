#pragma once

#include "eigen.hh"

namespace numerics {
template <int rows>
bool is_pd(const MatNd<rows, rows>& mat, const double eps = 1e-5) {
  const SelfAdjointEigenSolver<MatNd<rows, rows>> solver(mat);
  const double min_eigenvalue = solver.eigenvalues()[0];
  return (min_eigenvalue > eps);
}
}  // namespace numerics