#pragma once

#include "control/vanes_generated.hh"
#include "embedded/servo_bq/set_servo_message.hh"

namespace jet {
namespace control {

// Generate a sendable servo message from a quadraframe status
SetServoMessage create_servo_command(const QuadraframeStatus& qframe_status);

QuadraframeStatus create_quadraframe_status(const SetServoMessage& servo_message);

}  // namespace control
}  // namespace jet
