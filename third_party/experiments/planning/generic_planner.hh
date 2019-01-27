#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include "numerics/optimize.hh"
#include "numerics/wrap_optimizer.hh"

namespace planning {

namespace {
using VecX = Eigen::VectorXd;

}  // namespace

// Concept: Dynamics
// Dynamics must be a function, dynamics of the form
// State <- dynamics(State, Vector(U_DIM), dt);
//
template <typename State, int U_DIM>
class GenericPlanner {
 public:
  using ControlVec = VecNd<U_DIM>;
  using Cost = std::function<double(const State&, const ControlVec& u, int t)>;
  using Dynamics =
      std::function<State(const State&, const ControlVec& u, const double dt)>;

  GenericPlanner(const Dynamics& dynamics, const Cost& cost)
      : dynamics_(dynamics), cost_(cost) {
  }

  static constexpr int HORIZON = 25;
  static constexpr double dt = 0.25;

  //
  // Form the problem
  //
  struct StateControl {
    State state;
    ControlVec control;
  };

  std::vector<StateControl> plan(
      const State& x0, const std::vector<ControlVec>& initialization = {}) const {
    const auto opt_problem = build_optimization_problem(x0);
    const VecX soln = optimize(opt_problem, initialization);
    std::vector<StateControl> full_soln;

    {  // TODO: Factor this out
      State xt = x0;
      for (int t = 0; t < HORIZON; ++t) {
        const ControlVec ut = soln.segment(t, U_DIM);
        full_soln.push_back({xt, ut});
        xt = dynamics_(xt, ut, dt);
      }
    }
    return full_soln;
  }

 private:
  numerics::OptimizationProblem build_optimization_problem(const State& x0) const {
    // Forward shooting
    const auto opt_cost = [this, x0](const VecX& u) {
      double total_cost = 0.0;
      State xt = x0;

      for (int t = 0; t < HORIZON; ++t) {
        const ControlVec ut = u.segment(t, U_DIM);
        xt = dynamics_(xt, ut, dt);
        total_cost += cost_(xt, ut, t);
      }
      return total_cost;
    };

    const auto wrapped_cost = numerics::wrap_numerical_grad(opt_cost);

    const numerics::OptimizationProblem problem{wrapped_cost};
    return problem;
  }

  VecX optimize(const numerics::OptimizationProblem& problem,
                const std::vector<ControlVec>& initialization) const {
    VecX u_init = VecX::Ones(HORIZON * U_DIM);
    if (!initialization.empty()) {
      for (std::size_t t = 0; t < initialization.size(); ++t) {
        u_init.segment(t, U_DIM) = initialization.at(t);
      }
    }
    const numerics::OptimizationState initialization_vec{u_init};

    const auto result = numerics::optimize<numerics::ObjectiveMethod::kGradientDescent,
                                           numerics::ConstraintMethod::kAugLag>(
        initialization_vec, problem);
    return result.x;
  }

  const Dynamics dynamics_;
  const Cost cost_;
};

}  // namespace planning

// #include "planning/generic_planner.hh"
