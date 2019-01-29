#include "planning/jet/filter_sim.hh"

#include "eigen_helpers.hh"

namespace planning {
namespace jet {

estimation::jet_filter::State kf_state_from_xlqr_state(const State& x,
                                                       const Controls& u,
                                                       const Parameters& z) {
  estimation::jet_filter::State kf_state;
  const SE3 world_from_body = SE3(x.R_world_from_body, x.x);

  kf_state.T_body_from_world = world_from_body.inverse();

  const jcc::Vec3 dR_dt = x.w;
  const jcc::Vec3 dx_dt = x.v;

  const jcc::Vec3 net_force = x.R_world_from_body * (jcc::Vec3::UnitZ() * x.throttle_pct);
  const jcc::Vec3 dv_dt = (z.external_force + net_force) / z.mass;
  const jcc::Vec3 dw_dt = u.q;

  kf_state.eps_dot = jcc::vstack(dx_dt, (-dR_dt).eval());
  kf_state.eps_ddot = jcc::vstack(dv_dt, dw_dt);
  return kf_state;
}

}  // namespace jet
}  // namespace planning