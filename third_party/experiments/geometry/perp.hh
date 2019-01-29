#pragma once

#include "eigen.hh"

namespace geometry {

template <int ROWS>
VecNd<ROWS> perp(const VecNd<ROWS>& vec) {
  using Vec = VecNd<ROWS>;

  //
  // Gram-schmidt
  //

  int min_coeff_ind;
  vec.minCoeff(&min_coeff_ind);

  const Vec vec_u = vec.normalized();

  const Vec unit = Vec::Unit(min_coeff_ind);
  return (unit - (vec_u * unit.dot(vec_u))).normalized();
}

Eigen::VectorXd perp(const Eigen::VectorXd& vec);

template <int ROWS>
bool is_perp(const VecNd<ROWS>& a, const VecNd<ROWS>& b, double max_abs_cos = 1e-9) {
  if (std::abs(a.dot(b)) > max_abs_cos) {
    return false;
  }
  return true;
}
}  // namespace geometry
