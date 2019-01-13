#pragma once

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {
namespace control {

namespace detail {
SE3 compute_T_vane_unit_from_aerodynamic_frame();
}  // namespace detail

// Vane static configuration
struct VaneConfiguration {
  SE3 T_vane_unit_from_zero = detail::compute_T_vane_unit_from_aerodynamic_frame();
  double offset_vane_from_servo_rad = 0.0;

  double gear_ratio_servo_from_vane = 5.33;

  double vane_area_m2 = 20e-3 * 15e-3;

  double k = 1.0 / 3.0;
  double cd0 = 0.05;
};

// Jet static configuration
struct JetConfiguration {
  double v_exhaust_full_throttle_mps = 442.0;

  // Density of exhaust gas at 100 kPa, 900 K
  double rho_exhaust_kg_p_m3 = 0.39;
};

// Jet varying parameters
struct JetStatus {
  // Dimensionless: throttle ∈ [0, 1]
  double throttle;
};

struct QuadraframeConfiguration {
  const jcc::Vec3 quadraframe_from_vane_unit_0_translation =
      jcc::Vec3(0.0, 46.7e-3, -9.5e-3);

  SE3 com_from_vane_unit_0 =
      SE3(jcc::exp_z(0.0).so3(), quadraframe_from_vane_unit_0_translation);
  SE3 com_from_vane_unit_1 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_0;
  SE3 com_from_vane_unit_2 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_1;
  SE3 com_from_vane_unit_3 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_2;
};
}  // namespace control
}  // namespace jet