#include "numerics/line_search.hh"

namespace numerics {

LinesearchResult line_search(const OptimizationState& current_state,
                             const Eigen::VectorXd& direction,
                             const OptimizationProblem& problem) {
  Eigen::VectorXd best_x = current_state.x;
  double best_cost_so_far = problem.objective(best_x, nullptr, nullptr);

  bool did_decrease = false;
  // for (const double alpha : {0.001, 0.2, 0.5, 1.0, 5.0, 9.0, 25.0}) {
  for (const double alpha : {0.001, 0.2, 0.5, 1.0}) {
    const Eigen::VectorXd evaluation_pt = current_state.x - (alpha * direction);
    const double cost_at_alpha = problem.objective(evaluation_pt, nullptr, nullptr);

    if (cost_at_alpha < best_cost_so_far) {
      best_cost_so_far = cost_at_alpha;
      best_x = evaluation_pt;
      did_decrease = true;
    }
  }
  const LinesearchResult result(
      {.valid = did_decrease, .cost = best_cost_so_far, .best_x = best_x});
  return result;
}

}  // namespace numerics