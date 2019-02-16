#include "control/control_utilities.hh"

namespace jet {
namespace control {

SO3 target_from_joy(double joy_pitch_axis, double joy_roll_axis, double joy_yaw_axis) {
  double y_gain = 0.1;
  double x_gain = 0.1;
  double z_gain = 1.0;
  // This is what jake thinks the operator will expect most reasonably
  const SO3 target_from_world = SO3::exp(jcc::Vec3(y_gain * joy_roll_axis, x_gain * joy_pitch_axis, 0.0)) *
                                jcc::exp_z(z_gain * joy_yaw_axis).so3();
  return target_from_world;
}

}  // namespace control
}  // namespace jet
