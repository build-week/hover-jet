#pragma once

#include "estimation/observation_model.hh"

#include "numerics/group_diff.hh"
#include "numerics/is_symmetric.hh"

// TODO
#include <iostream>

namespace estimation {

template <typename State, typename Observation>
FilterStateUpdate<State> ObservationModel<State, Observation>::generate_update(
    const FilterState<State>& xp, const Observation& z) const {
  using ObservationInfo = MatNd<Observation::DIM, Observation::DIM>;

  const ObsVec innovation = error_model_(xp.x, z);

  const auto held_error_model = [this, &z](const State& x) -> ObsVec {
    const ObsVec y = error_model_(x, z);
    return y;
  };

  const MatNd<Observation::DIM, State::DIM> H =
      -numerics::group_jacobian<Observation::DIM, State>(xp.x, held_error_model);

  const ObservationInfo S = (H * xp.P * H.transpose()) + cov_;

  assert(numerics::is_symmetric(xp.P));

  const Eigen::LLT<ObservationInfo> S_llt(S);
  if (S_llt.info() != Eigen::Success) {
    std::cerr << "LLT solve was degenerate" << std::endl;
    assert(false);
  }

  using StateInfo = MatNd<State::DIM, State::DIM>;

    const MatNd<State::DIM, Observation::DIM> PHt = xp.P * H.transpose();
    const StateVec update = PHt * S_llt.solve(innovation);
    const StateInfo P_new = (StateInfo::Identity() - (PHt * S_llt.solve(H))) * xp.P;

  return FilterStateUpdate<State>{update, P_new};
}

}  // namespace estimation
