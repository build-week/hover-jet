#include <iostream>

#include "numerics/nt_gauss_newton.hh"

namespace numerics {

MinimizationResult nt_gauss_newton(const Eigen::VectorXd &initialization,
                                   const GaussNewtonProblem &problem,
                                   const NtGaussNewtonConfiguration &config) {
  using Information = Eigen::MatrixXd;
  int problem_dim = initialization.size();
  int in_dim = initialization.size();

  InVec jti = InVec::Zero(in_dim);
  for (int k = 0; k < opt_cfg.max_iterations; ++k) {
    Information jtj = Information::Zero(problem_dim, problem_dim);
    InVec jti = InVec::Zero();

    //
    // Accumulate System
    //
    ErrorJacobian jac;
    for (const auto &func : problem.error_functions) {
      jac.setZero();
      const OutVec innovation = func(x, &jac);
      jtj += jac.transpose() * jac;
      jti += jac.transpose() * innovation;
    }

    Information hessian = Information::Zero();
    InVec gradient = InVec::Zero();
    for (const auto &func : problem.hessian_functions) {
      hess.setZero();
      const OutVec innovation = func(x, &gradient, &hess);
      jtj += hess;
      jti += gradient;
    }
  }
}
