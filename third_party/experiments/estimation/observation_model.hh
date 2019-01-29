#pragma once

#include "eigen.hh"

#include "estimation/filter_state.hh"

// TODO
#include <iostream>

namespace estimation {

template <typename State, typename Observation>
class ObservationModel {
 public:
  using StateVec = VecNd<State::DIM>;
  using ObsVec = VecNd<Observation::DIM>;

  using ErrorModel = std::function<ObsVec(const State&, const Observation&)>;
  using LogLikelihood = std::function<double(const State&, const Observation& z)>;

  ObservationModel(const ErrorModel& error_model,
                   const MatNd<Observation::DIM, Observation::DIM>& cov)
      : error_model_(error_model), cov_(cov) {
  }

  // This return z [-] h(x)
  ObsVec error(const State& x, const Observation& z) const {
    return error_model_(x, z);
  }

  FilterStateUpdate<State> generate_update(const FilterState<State>& xp,
                                           const Observation& z) const;

  FilterStateUpdate<State> operator()(const FilterState<State>& xp,
                                      const Observation& z) const {
    return generate_update(xp, z);
  }

 private:
  const ErrorModel error_model_;

  // "R" by convention
  const MatNd<Observation::DIM, Observation::DIM> cov_;

  LogLikelihood log_likelihood_;
};

}  // namespace estimation