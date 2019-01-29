#pragma once

#include "planning/xlqr_problem.hh"

#include <iostream>
#include <limits>

namespace planning {

template <typename _Prob>
typename XlqrProblem<_Prob>::Solution XlqrProblem<_Prob>::solve(
    const typename XlqrProblem<_Prob>::State x0,
    const typename XlqrProblem<_Prob>::Solution& initialization) const {
  Solution traj0;
  if (initialization.x.empty()) {
    traj0.x.resize(prob_.horizon());
    traj0.x[0] = x0;
    traj0.u.resize(prob_.horizon() - 1, ControlVec::Zero());
  } else {
    assert(initialization.x.size() == prob_.horizon() &&
           (initialization.u.size() == prob_.horizon() - 1));
    traj0 = initialization;
  }

  Solution current_soln;
  shoot(traj0, {}, -1.0, &current_soln);

  constexpr int MAX_ITERS = 7;
  for (int iter = 0; iter < MAX_ITERS; ++iter) {
    const auto lqr_soln = ricatti(current_soln);
    current_soln = line_search(current_soln, lqr_soln);
  }
  return current_soln;
}

template <typename _Prob>
double XlqrProblem<_Prob>::shoot(const typename XlqrProblem<_Prob>::Solution& soln,
                                 const typename XlqrProblem<_Prob>::LqrSolution& lqr_soln,
                                 double alpha,
                                 Solution* out_soln) const {
  if (out_soln) {
    out_soln->x.clear();
    out_soln->u.clear();
    out_soln->x.reserve(soln.x.size());
    out_soln->u.reserve(soln.u.size());
  }

  State x = soln.x.front();

  double cost = 0.0;
  for (std::size_t t = 0; t < soln.u.size(); ++t) {
    const StateVec error = prob_.delta_vec(x, soln.x.at(t));

    // TEMP: HACK
    ControlVec optimized_u;
    if (alpha == -1.0) {
      optimized_u = ControlVec::Zero();
    } else {
      const ControlVec feedback_u = lqr_soln.at(t).K * error;
      const ControlVec feedforward_u = alpha * lqr_soln[t].k;
      optimized_u = feedback_u + feedforward_u;
    }
    const ControlVec executed_u = soln.u[t] + optimized_u;
    cost += prob_.cost(x, executed_u, t);

    if (out_soln) {
      out_soln->x.push_back(x);
      out_soln->u.push_back(executed_u);
    }

    x = prob_.dynamics(x, executed_u);
  }
  if (out_soln) {
    out_soln->x.push_back(x);
  }
  cost += prob_.cost(x, ControlVec::Zero(), prob_.horizon());

  return cost;
}

template <typename _Prob>
typename XlqrProblem<_Prob>::Solution XlqrProblem<_Prob>::line_search(
    const typename XlqrProblem<_Prob>::Solution& soln,
    const typename XlqrProblem<_Prob>::LqrSolution& lqr_soln) const {
  double best_cost = std::numeric_limits<double>::max();
  double best_alpha = -1.0;
  for (double alpha : {0.01, 0.1, 0.125, 0.25, 0.5, 1.0}) {
    const double cost = shoot(soln, lqr_soln, alpha);
    if (cost < best_cost) {
      best_cost = cost;
      best_alpha = alpha;
    }
  }

  assert(best_alpha > 0.0);

  Solution out_soln;
  shoot(soln, lqr_soln, best_alpha, &out_soln);
  return out_soln;
}

template <typename _Prob>
typename XlqrProblem<_Prob>::LqrSolution XlqrProblem<_Prob>::ricatti(
    const typename XlqrProblem<_Prob>::Solution& soln) const {
  LqrSolution lqr_soln;
  lqr_soln.resize(soln.u.size());

  using StateHess = MatNd<Prob::X_DIM, Prob::X_DIM>;
  using ControlHess = MatNd<Prob::U_DIM, Prob::U_DIM>;
  using StateControlHessBlock = MatNd<Prob::U_DIM, Prob::X_DIM>;

  const Derivatives final_derivatives =
      cost_diffs_(soln.x.back(), ControlVec::Zero(), prob_.horizon());
  StateHess Vxx = final_derivatives.Q;
  VecNd<Prob::X_DIM> Vx = final_derivatives.g_x;

  const int ctrl_size = static_cast<int>(soln.u.size());
  for (int k = ctrl_size - 1; k >= 0; --k) {
    const State& x = soln.x.at(k);
    const ControlVec& u = soln.u.at(k);
    const Derivatives diffs = cost_diffs_(x, u, k);

    const StateVec Qx = diffs.g_x + diffs.A.transpose() * Vx;
    const ControlVec Qu = diffs.g_u + diffs.B.transpose() * Vx;

    const StateHess Qxx = diffs.Q + diffs.A.transpose() * Vxx * diffs.A;
    const ControlHess Quu = diffs.R + diffs.B.transpose() * Vxx * diffs.B;
    const StateControlHessBlock Qux = diffs.N + diffs.B.transpose() * Vxx * diffs.A;

    // Safe LLT computation
    const Eigen::LLT<ControlHess> llt(Quu);
    if (llt.info() != Eigen::Success) {
      std::cout << "LLT solve was degenerate at state " << k << std::endl;
      assert((llt.info() == Eigen::Success));
    }

    // Control policy is trivial
    lqr_soln[k].K = -llt.solve(Qux);
    lqr_soln[k].k = -llt.solve(Qu);

    // Compute cost-to-go for next step state (Schur complement)
    Vx = Qx - lqr_soln[k].K.transpose() * Quu * lqr_soln[k].k;
    Vxx = Qxx - lqr_soln[k].K.transpose() * Quu * lqr_soln[k].K;
    Vxx = (Vxx + Vxx.transpose()) * 0.5;
  }
  return lqr_soln;
}

}  // namespace planning
