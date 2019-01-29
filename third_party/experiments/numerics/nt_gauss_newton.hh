#pragma once

//%ignore

#include "eigen.hh"

#include <vector>

namespace numerics {

// Same type as above, but one is expected to return a hessian
using HessianFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &, Eigen::MatrixXd *)>;
using ErrorFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd &, Eigen::MatrixXd *)>;

struct MinimizationResult {
  Eigen::VectorXd solution;
};

struct GaussNewtonProblem {
  std::vector<ErrorFunction> error_functions;
  std::vector<HessianFunction> hessian_functions;
};

struct NtGaussNewtonConfiguration {
  int max_iterations = 5;
  double levenberg_mu = 1e-3;
};

// I think when I wrote this, I named it "nt" for "not-template"
// But as is the great crisis of all things, I didn't write this comment when I wrote the function,
// so I have no idea what the real reason for nt_ is
MinimizationResult nt_gauss_newton(const Eigen::VectorXd &initialization,
                                   const GaussNewtonProblem &problem,
                                   const NtGaussNewtonConfiguration &config = {});
}
