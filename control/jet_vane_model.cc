#include "control/jet_vane_model.hh"

#include "third_party/experiments/eigen_helpers.hh"

namespace jet {
namespace control {

namespace detail {
SE3 compute_T_vane_unit_from_vane_default() {
  const jcc::Vec3 vane_center_of_pressure_vane_unit_frame = jcc::Vec3(0.0, -0.030, 0.0);

  const SO3 R_intermediate_from_aerodynamic_frame =
      SO3::exp(jcc::Vec3(0.0, M_PI * 0.5, 0.0));
  const SO3 R_vane_unit_from_intermediate = SO3::exp(jcc::Vec3(0.0, 0.0, -M_PI * 0.5));
  const SO3 R_vane_unit_from_aerodynamic_frame =
      R_vane_unit_from_intermediate * R_intermediate_from_aerodynamic_frame;

  const SE3 T_vane_unit_from_vane_default =
      SE3(R_vane_unit_from_aerodynamic_frame, vane_center_of_pressure_vane_unit_frame);
  return T_vane_unit_from_vane_default;
}
}  // namespace detail

namespace {
double approximate_v_exhaust_mps(const double throttle, const JetConfiguration& params) {
  // Matt says ~ sqrt(throttle)
  return std::sqrt(throttle) * params.v_exhaust_full_throttle_mps;
}
}  // namespace

Wrench vane_unit_wrench(const double commanded_servo_angle_rad,
                        const JetStatus& jet_status,
                        const VaneConfiguration& vane_cfg,
                        const JetConfiguration& jet_cfg) {
  const double v_exhaust_mps = approximate_v_exhaust_mps(jet_status.throttle, jet_cfg);

  // TODO(jake, matt): Make this a jet status element, or otherwise computable
  constexpr double SWIRL_ANGLE = 0.0;

  // commanded_vane_angle_rad is the angle of the vane chord-line w.r.t the vehicle axis
  // of symmetry
  const double commanded_vane_angle_rad =
      (commanded_servo_angle_rad / vane_cfg.gear_ratio_servo_from_vane) +
      vane_cfg.offset_vane_from_servo_rad;
  const double angle_of_attack_rad = -commanded_vane_angle_rad + SWIRL_ANGLE;

  const double v_exhaust_mps_sq = v_exhaust_mps * v_exhaust_mps;
  const double dynamic_pressure_Pa = 0.5 * v_exhaust_mps_sq * jet_cfg.rho_exhaust_kg_p_m3;

  // lift_coeff: is unitless
  const double lift_coeff = angle_of_attack_rad * 2.0 * M_PI;
  const double lift_force_at_vane_N =
      dynamic_pressure_Pa * lift_coeff * vane_cfg.vane_area_m2;

  const double drag_coeff = vane_cfg.cd0 + (4.0 * M_PI * M_PI * vane_cfg.induced_drag_param *
                                            (angle_of_attack_rad * angle_of_attack_rad));
  const double drag_force_N = dynamic_pressure_Pa * drag_coeff * vane_cfg.vane_area_m2;
  // TODO: Come up with a strategy for verifying at runtime that we don't compute a
  // negative drag

  const jcc::Vec2 force_exhaust_velocity_frame(drag_force_N, lift_force_at_vane_N);

  // See the header for a description of the default frame
  const SO2 default_frame_from_exhaust_velocity = SO2::exp(SWIRL_ANGLE);
  const jcc::Vec2 force_vane_default_frame =
      default_frame_from_exhaust_velocity * force_exhaust_velocity_frame;
  const jcc::Vec3 force_vane_default_frame_3d =
      jcc::augment(force_vane_default_frame, 0.0);

  const Wrench wrench_default_frame({.force_N = force_vane_default_frame_3d,  //
                                     .torque_Nm = jcc::Vec3::Zero()});

  // The vane "unit" frame; as described in the CAD
  const Wrench wrench_vane_unit_frame =
      wrench_in_frame(vane_cfg.T_vane_unit_from_vane_default, wrench_default_frame);

  return wrench_vane_unit_frame;
}
}  // namespace control
}  // namespace jet
