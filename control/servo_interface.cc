#include "control/servo_interface.hh"

#include <cassert>

namespace jet {
namespace control {

SetServoMessage create_servo_command(const QuadraframeStatus& qframe_status) {
  SetServoMessage servo_msg;

  servo_msg.servo_indices.push_back(0);
  servo_msg.target_radians.push_back(-qframe_status.servo_0_angle_rad);

  servo_msg.servo_indices.push_back(1);
  servo_msg.target_radians.push_back(-qframe_status.servo_1_angle_rad);

  servo_msg.servo_indices.push_back(2);
  servo_msg.target_radians.push_back(-qframe_status.servo_2_angle_rad);

  servo_msg.servo_indices.push_back(3);
  servo_msg.target_radians.push_back(-qframe_status.servo_3_angle_rad);
  return servo_msg;
}

QuadraframeStatus create_quadraframe_status(const SetServoMessage& servo_msg) {
  assert(servo_msg.servo_indices.size() == 4u);
  assert(servo_msg.target_radians.size() == 4u);

  for (std::size_t k = 0; k < servo_msg.servo_indices.size(); ++k) {
    // Assert we have 0...3 and only 0...3
    assert(servo_msg.servo_indices.at(k) == static_cast<int>(k));
  }

  QuadraframeStatus qframe_status;
  qframe_status.servo_0_angle_rad = -servo_msg.target_radians[0];
  qframe_status.servo_1_angle_rad = -servo_msg.target_radians[1];
  qframe_status.servo_2_angle_rad = -servo_msg.target_radians[2];
  qframe_status.servo_3_angle_rad = -servo_msg.target_radians[3];
  return qframe_status;
}

}  // namespace control
}  // namespace jet
