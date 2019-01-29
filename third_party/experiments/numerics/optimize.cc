#include "numerics/optimize.hh"

#include "numerics/line_search.hh"

#include <limits>

namespace numerics {
namespace {
OptimizationState compute_lagrange_update(const OptimizationState& current_state,
                                          const OptimizationProblem& problem) {
  return {};
}

}  // namespace

template <>
OptimizationState optimize<ObjectiveMethod::kGradientDescent, ConstraintMethod::kAugLag>(
    const OptimizationState& initialization, const OptimizationProblem& problem) {
  OptimizationState iteration_state = initialization;

  constexpr int MAX_ITERS = 10000;
  for (int k = 0; k < MAX_ITERS; ++k) {
    detail::Vecx gradient;
    problem.objective(iteration_state.x, &gradient, nullptr);

    const auto lsr = line_search(iteration_state, gradient, problem);
    if (!lsr.valid) {
      // Could not minimize
      return iteration_state;
    }
    const double cost_ratio =
        (iteration_state.x - lsr.best_x).squaredNorm() / iteration_state.x.squaredNorm();

    iteration_state.x = lsr.best_x;
    constexpr double COST_CVG_RATIO = 1e-12;
    if (cost_ratio < COST_CVG_RATIO) {
      // Converged
      break;
    }
  }
  return iteration_state;
}
}  // namespace numerics
