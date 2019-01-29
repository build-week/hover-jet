#pragma once

#include "estimation/jet/jet_filter.hh"
#include "planning/jet/jet_dynamics.hh"

namespace planning {
namespace jet {

estimation::jet_filter::State kf_state_from_xlqr_state(const State& x,
                                                       const Controls& u,
                                                       const Parameters& z);

}  // namespace jet
}  // namespace planning