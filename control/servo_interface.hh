#pragma once

#include "embedded/servo_bq/set_servo_message.hh"

namespace jet {
namespace control {

// (Time-varying) status of the quadraframe
// A struct that comes from another control PR, the wrench-mapper
// TODO(jake): delete when [1] lands
//
// [1] https://github.com/build-week/hover-jet/pull/51
//
struct QuadraframeStatus {
  double servo_0_angle_rad;
  double servo_1_angle_rad;
  double servo_2_angle_rad;
  double servo_3_angle_rad;
};

// Generate a sendable servo message from a quadraframe status
SetServoMessage create_servo_command(const QuadraframeStatus& qframe_status);

QuadraframeStatus create_quadraframe_status(const SetServoMessage& servo_message);

}  // namespace control
}  // namespace jet
