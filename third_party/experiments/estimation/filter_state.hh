#pragma once

#include "eigen.hh"

#include "estimation/time_point.hh"

namespace estimation {
template <typename State>
struct FilterState {
  // The actual state
  State x;

  // State-covariance
  MatNd<State::DIM, State::DIM> P;

  // When this state was the estimate for the true state
  TimePoint time_of_validity;
};

template <typename State>
struct FilterStateUpdate {
  // The actual state
  VecNd<State::DIM> dx;

  // State-covariance
  MatNd<State::DIM, State::DIM> P_new;
};

}  // namespace estimation