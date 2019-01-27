#pragma once

#include "numerics/numdiff.hh"

namespace numerics {

// Compute jacobian of gradient for some stupid function
//
// Don't do any clever central difference approximation or anything, we're just going for
// broke here
template <int rows, typename Callable>
Eigen::Matrix<double, rows, rows> numerical_hessian(
    const Eigen::Matrix<double, rows, 1> &x,
    const Callable &fcn,
    const double feps = 1e-6) {
  const auto gradient_fnc = [fcn, feps](const Eigen::Matrix<double, rows, 1> &pt) {
    return numerical_gradient<rows>(pt, fcn, feps);
  };

  const auto hessian = numerical_jacobian<rows>(x, gradient_fnc, feps);
  return hessian;
}
}  // namespace numerics
