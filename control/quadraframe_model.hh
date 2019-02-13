#pragma once

#include "control/jet_vane_model.hh"
#include "control/wrench.hh"
#include "control/vanes_generated.hh"


namespace jet {
namespace control {

struct QuadraframeConfiguration {
  const jcc::Vec3 quadraframe_from_vane_unit_0_translation = jcc::Vec3(0.0, 46.7e-3, -9.5e-3);

  SE3 com_from_vane_unit_0 = SE3(jcc::exp_z(M_PI).so3(), quadraframe_from_vane_unit_0_translation);
  SE3 com_from_vane_unit_1 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_0;
  SE3 com_from_vane_unit_2 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_1;
  SE3 com_from_vane_unit_3 = jcc::exp_z(M_PI * 0.5) * com_from_vane_unit_2;
};

Wrench total_wrench_com_frame(const JetStatus& jet_status,
                              const QuadraframeStatus& quadraframe_status,
                              const VaneConfiguration& vane_cfg,
                              const JetConfiguration jet_cfg,
                              const QuadraframeConfiguration& quadraframe_cfg);

}  // namespace control
}  // namespace jet
