#pragma once

#include "eigen.hh"
#include "sophus.hh"

namespace planning {
namespace jet {
struct StateDelta {
  VecNd<3> R_world_from_body_error_log = VecNd<3>::Zero();
  double throttle_pct_error = 0.0;
  VecNd<3> x_error = VecNd<3>::Zero();
  VecNd<3> w_error = VecNd<3>::Zero();
  VecNd<3> v_error = VecNd<3>::Zero();
  static constexpr int DIM = 13;
};
struct Parameters {
  VecNd<3> unit_z = VecNd<3>::Zero();
  double mass = 0.0;
  VecNd<3> external_force = VecNd<3>::Zero();
  static constexpr int DIM = 7;
};
struct Controls {
  VecNd<3> q = VecNd<3>::Zero();
  double throttle_dot = 0.0;
  static constexpr int DIM = 4;
};
struct State {
  SO3 R_world_from_body = SO3();
  double throttle_pct = 0.0;
  VecNd<3> x = VecNd<3>::Zero();
  VecNd<3> w = VecNd<3>::Zero();
  VecNd<3> v = VecNd<3>::Zero();
  static constexpr int DIM = 13;
};
struct StateDot {
  VecNd<3> w = VecNd<3>::Zero();
  double throttle_dot = 0.0;
  VecNd<3> v = VecNd<3>::Zero();
  VecNd<3> q = VecNd<3>::Zero();
  VecNd<3> a = VecNd<3>::Zero();
  static constexpr int DIM = 13;
};
VecNd<13> to_vector(const StateDelta &in_grp);
VecNd<13> compute_delta(const State &a, const State &b);
State rk4_integrate(const State &Q, const Controls &U, const Parameters &Z,
                    const double h);
StateDelta from_vector(const VecNd<13> &in_vec);
State apply_delta(const State &a, const VecNd<13> &delta);
Controls from_vector(const VecNd<4> &in_vec);
VecNd<4> to_vector(const Controls &in_grp);
} // namespace jet
} // namespace planning