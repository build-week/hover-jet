#pragma once

#include "numerics/num_hessian.hh"
#include "numerics/numdiff.hh"

#include "planning/problem.hh"

namespace planning {

template <typename _Prob>
class Differentiator {
  using Prob = _Prob;
  using ControlVec = typename Prob::ControlVec;
  using StateVec = typename Prob::StateVec;
  using State = typename Prob::State;

 public:
  // TODO: Generate this whole shebang
  Differentiator(const Prob &prob) : prob_(prob) {
  }

  MatNd<Prob::X_DIM, Prob::X_DIM> state_jacobian(const State &x,
                                                 const ControlVec &u) const {
    const auto f_x = [this, &x, &u](const StateVec &dx) {
      const State xplus_dx = prob_.apply_delta(x, dx);
      const State xn = prob_.dynamics(xplus_dx, u);
      return prob_.delta_vec(xn, prob_.dynamics(x, u));
    };
    return numerics::numerical_jacobian<Prob::X_DIM>(StateVec::Zero().eval(), f_x);
  }

  MatNd<Prob::X_DIM, Prob::U_DIM> control_jacobian(const State &x,
                                                   const ControlVec &u) const {
    const auto f_u = [this, &x, &u](const ControlVec &du) {
      return prob_.delta_vec(prob_.dynamics(x, u + du), prob_.dynamics(x, u));
    };
    return numerics::numerical_jacobian<Prob::X_DIM>(ControlVec::Zero().eval(), f_u);
  }

  MatNd<Prob::X_DIM, Prob::X_DIM> state_hessian(const State &x,
                                                const ControlVec &u,
                                                int t) const {
    const auto c_x = [this, t, &x, &u](const StateVec &dx) {
      const State xplus_dx = prob_.apply_delta(x, dx);
      return prob_.cost(xplus_dx, u, t);
    };
    return numerics::numerical_hessian(StateVec::Zero().eval(), c_x);
  }

  VecNd<Prob::X_DIM> state_gradient(const State &x, const ControlVec &u, int t) const {
    const auto c_x = [this, t, &x, &u](const StateVec &dx) {
      const State xplus_dx = prob_.apply_delta(x, dx);
      return prob_.cost(xplus_dx, u, t);
    };
    return numerics::numerical_gradient(StateVec::Zero().eval(), c_x);
  }

  MatNd<Prob::U_DIM, Prob::U_DIM> control_hessian(const State &x,
                                                  const ControlVec &u,
                                                  int t) const {
    const auto c_u = [this, t, &x, &u](const ControlVec &du) {
      //
      return prob_.cost(x, u + du, t);
    };
    return numerics::numerical_hessian(ControlVec::Zero().eval(), c_u);
  }

  VecNd<Prob::U_DIM> control_gradient(const State &x, const ControlVec &u, int t) const {
    const auto c_u = [this, t, &x, &u](const ControlVec &du) {
      //
      return prob_.cost(x, u + du, t);
    };
    return numerics::numerical_gradient(ControlVec::Zero().eval(), c_u);
  }

 private:
  Prob prob_;
};

}  // namespace planning
