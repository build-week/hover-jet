#pragma once

#include "control/servo_interface.hh"

namespace jet {
namespace control {

SetServoMessage create_servo_command(const QuadraframeStatus& qframe_status) {
  SetServoMessage servo_msg;

  servo_msg.servo_indices.push_back(0);
  servo_msg.target_radians.push_back(qframe_status.servo_0_angle_rad);

  servo_msg.servo_indices.push_back(1);
  servo_msg.target_radians.push_back(qframe_status.servo_1_angle_rad);

  servo_msg.servo_indices.push_back(2);
  servo_msg.target_radians.push_back(qframe_status.servo_2_angle_rad);

  servo_msg.servo_indices.push_back(3);
  servo_msg.target_radians.push_back(qframe_status.servo_3_angle_rad);
  return servo_msg;
}

}  // namespace control
}  // namespace jet
