#pragma once

#include "estimation/jet/jet_rk4.hh"

namespace estimation {
namespace jet_filter {

struct FiducialMeasurement {
  int fiducial_id = -1;
  SE3 T_fiducial_from_camera;
  static constexpr int DIM = 6;
};

// TODO generate this
FiducialMeasurement observe_fiducial(const State& x, const Parameters& p);

// TODO: generate this
VecNd<6> fiducial_error_model(const State& x,
                              const FiducialMeasurement& z,
                              const Parameters& p);

}  // namespace jet_filter
}  // namespace estimation
