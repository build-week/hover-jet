#include "numerics/gradient_descent.hh"

namespace numerics {

Eigen::VectorXd minimize_gradient_descent(const Eigen::VectorXd &initialization,
                                          const SimpleCostFunction &cost_func,
                                          const GradientDescentConfig &cfg) {
  // Later, write bfgs
  double best_cost = cost_func(initialization, nullptr);

  Eigen::VectorXd x = initialization;

  int k = 0;
  bool done = false;
  while (!done) {
    k += 1;

    if (cfg.max_iters != -1 && k > cfg.max_iters) {
      done = true;
    }

    if (done) {
      break;
    }

    Eigen::VectorXd gradient;
    cost_func(x, &gradient);

    // Do a linesearch
    Eigen::VectorXd best_x = x;
    for (int j = -6; j < 10; ++j) {
      const double alpha = 1.3 * std::pow(0.5, j);
      const Eigen::VectorXd test_x = x - (alpha * gradient);
      const double cost = cost_func(test_x, nullptr);
      if (cost < best_cost) {
        best_cost = cost;
        best_x = test_x;
      }
    }

    const double cost_ratio = (x - best_x).squaredNorm() / x.squaredNorm();
    if (cost_ratio < cfg.cost_convergence_ratio) {
      done = true;
    }

    x = best_x;
  }

  return x;
}
}  // namespace numerics
