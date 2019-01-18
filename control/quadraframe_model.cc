#include "control/quadraframe_model.hh"

namespace jet {
namespace control {

Wrench total_wrench_com_frame(const JetStatus& jet_status,
                             const QuadraframeStatus& quadraframe_status,
                             const VaneConfiguration& vane_cfg,
                             const JetConfiguration jet_cfg,
                             const QuadraframeConfiguration& quadraframe_cfg) {
  const Wrench wrench_vane_0 = vane_unit_wrench(quadraframe_status.servo_0_angle_rad, jet_status, vane_cfg, jet_cfg);
  const Wrench wrench_vane_1 = vane_unit_wrench(quadraframe_status.servo_1_angle_rad, jet_status, vane_cfg, jet_cfg);
  const Wrench wrench_vane_2 = vane_unit_wrench(quadraframe_status.servo_2_angle_rad, jet_status, vane_cfg, jet_cfg);
  const Wrench wrench_vane_3 = vane_unit_wrench(quadraframe_status.servo_3_angle_rad, jet_status, vane_cfg, jet_cfg);

  const Wrench total_wrench = wrench_in_frame(quadraframe_cfg.com_from_vane_unit_0, wrench_vane_0) +
                              wrench_in_frame(quadraframe_cfg.com_from_vane_unit_1, wrench_vane_1) +
                              wrench_in_frame(quadraframe_cfg.com_from_vane_unit_2, wrench_vane_2) +
                              wrench_in_frame(quadraframe_cfg.com_from_vane_unit_3, wrench_vane_3);
  return total_wrench;
}

}  // namespace control
}  // namespace jet