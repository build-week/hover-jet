#pragma once

#include "control/jet_vane_model.hh"
#include "control/wrench.hh"

namespace jet {
namespace control {

struct QuadraframeStatus {
  // This is a [TODO(Jake), check] right-handed rotation about the servo shaft
  double servo_1_angle;
  double servo_2_angle;
  double servo_3_angle;
  double servo_4_angle;
};

struct QuadraframeConfiguration {
  const jcc::Vec3 quadraframe_from_vane_unit_1_translation = jcc::Vec3(0.0, 46.7e-3, -9.5e-3);

  SE3 com_from_vane_unit_1 = SE3(jcc::exp_z(0.0).so3(), quadraframe_from_vane_unit_1_translation);
  SE3 com_from_vane_unit_2 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_1;
  SE3 com_from_vane_unit_3 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_2;
  SE3 com_from_vane_unit_4 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_3;
};

Wrench total_force_com_frame(const JetStatus& jet_status,
                             const QuadraframeStatus& quadraframe_status,
                             const VaneConfiguration& vane_cfg,
                             const JetConfiguration jet_cfg,
                             const QuadraframeConfiguration& quadraframe_cfg);

}  // namespace control
}  // namespace jet