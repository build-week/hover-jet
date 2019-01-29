#pragma once

#include "eigen.hh"

namespace jcc {
namespace fluids {
namespace plane {

struct SimulationConfig {
  // Pressure constant
  double rho = 1e3;

  // Viscosity
  double nu = 0.9;

  // Grid spacing
  double dx = 1e-2;

  // Time step
  double dt = 1e-5;
};

struct SimulationState {
  Eigen::MatrixXd pressure_field;
  std::array<Eigen::MatrixXd, 2> velocity_field;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct JacobiConfig {
  double alpha = -1e-1;
  double beta = 4.0;
  int max_iters = -1;
  // Convergence criteria for jacobi iteration
  double min_rel_delta = 1e-6;
};

// Solve a poisson equation
//
// That is:
// Generate an image `x` such that Δ(x) = b
//
Eigen::MatrixXd solve_poisson_jacobi_image(const Eigen::MatrixXd& x0,
                                           const Eigen::MatrixXd& b,
                                           const JacobiConfig& config);

std::array<Eigen::MatrixXd, 2> compute_advection(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig& config);

std::array<Eigen::MatrixXd, 2> compute_pressure(const Eigen::MatrixXd pressure_field, const SimulationConfig& cfg);

std::array<Eigen::MatrixXd, 2> compute_diffusion(const std::array<Eigen::MatrixXd, 2> velocity_field,
                                                 const SimulationConfig& cfg);

SimulationState compute_projection(const SimulationState& state, const SimulationConfig& cfg);

}  // namespace plane
}  // namespace fluids
}  // namespace jcc