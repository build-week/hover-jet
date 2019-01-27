#pragma once

#include "planning/problem.hh"
namespace planning {

template <typename _Prob>
class XlqrProblem {
  using Prob = _Prob;
  using ControlVec = typename Prob::ControlVec;
  using StateVec = typename Prob::StateVec;
  using State = typename Prob::State;
  using Derivatives = typename Prob::Derivatives;

 public:
  XlqrProblem(const Prob& prob, const typename Prob::CostDiffs& cost_diffs)
      : prob_(prob), cost_diffs_(cost_diffs) {
  }

  struct Solution {
    std::vector<State> x;
    std::vector<ControlVec> u;
  };

  Solution solve(const State x0, const Solution& initialization = {}) const;

 private:
  struct LqrFeedback {
    // Feedback
    MatNd<Prob::U_DIM, Prob::X_DIM> K = MatNd<Prob::U_DIM, Prob::X_DIM>::Zero();

    // Feed-forward
    ControlVec k = ControlVec::Zero();
  };

  using LqrSolution = StdVector<LqrFeedback>;

  double shoot(const Solution& soln,
               const LqrSolution& lqr_soln,
               double alpha,
               Solution* out_soln = nullptr) const;

  Solution line_search(const Solution& soln, const LqrSolution& lqr_soln) const;

  LqrSolution ricatti(const Solution& soln) const;

  Prob prob_;
  typename Prob::CostDiffs cost_diffs_;
};

}  // namespace planning
