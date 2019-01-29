#pragma once

#include "eigen.hh"

namespace planning {

// X_DIM is in the dimension of *differences* in state
template <int _X_DIM, int _U_DIM, int _L_DIM = 0>
struct Dimensions {
  // State
  static constexpr int X_DIM = _X_DIM;
  // Control
  static constexpr int U_DIM = _U_DIM;
  // Lambda
  static constexpr int L_DIM = _L_DIM;
};

template <typename Dim>
struct GenericDerivatives {
  MatNd<Dim::X_DIM, Dim::X_DIM> Q;
  MatNd<Dim::U_DIM, Dim::U_DIM> R;
  VecNd<Dim::X_DIM> g_x;
  VecNd<Dim::U_DIM> g_u;

  MatNd<Dim::U_DIM, Dim::X_DIM> N = MatNd<Dim::U_DIM, Dim::X_DIM>::Zero();

  MatNd<Dim::X_DIM, Dim::X_DIM> A;
  MatNd<Dim::X_DIM, Dim::U_DIM> B;
};

template <typename Dim, typename _State>
class Problem {
 public:
  static constexpr int X_DIM = Dim::X_DIM;
  static constexpr int U_DIM = Dim::U_DIM;

  using State = _State;
  using StateVec = VecNd<X_DIM>;
  using ControlVec = VecNd<U_DIM>;
  using Derivatives = GenericDerivatives<Dim>;

  using Cost = std::function<double(const State&, const ControlVec& u, int t)>;
  using CostDiffs = std::function<Derivatives(const State&, const ControlVec& u, int t)>;
  using Dynamics =
      std::function<State(const State&, const ControlVec& u, const double dt)>;

  using DeltaVec = std::function<StateVec(const State&, const State&)>;
  using ApplyDelta = std::function<State(const State&, const StateVec&)>;

  Problem(const Cost& cost,
          const Dynamics& dynamics,
          const DeltaVec& delta,
          const ApplyDelta& apply_delta,
          std::size_t horizon,
          double dt)
      : cost_(cost),
        dynamics_(dynamics),
        delta_(delta),
        apply_delta_(apply_delta),
        horizon_(horizon),
        dt_(dt) {
  }

  double cost(const State& x, const ControlVec& u, const int t) const {
    return cost_(x, u, t);
  }

  State dynamics(const State& x, const ControlVec& u) const {
    return dynamics_(x, u, dt_);
  }

  StateVec delta_vec(const State& a, const State& b) const {
    return delta_(a, b);
  }
  State apply_delta(const State& a, const StateVec& b) const {
    return apply_delta_(a, b);
  }

  std::size_t horizon() const {
    return horizon_;
  }

 private:
  Cost cost_;
  // CostDiffs cost_diffs;
  Dynamics dynamics_;
  DeltaVec delta_;
  ApplyDelta apply_delta_;

  std::size_t horizon_;
  double dt_;
};

}  // namespace planning
