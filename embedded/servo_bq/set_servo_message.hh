#pragma once

#include "infrastructure/comms/schemas/message.hh"
// #include "infrastructure/comms/serialization/serialization_macros.hh"
#include <vector>

namespace jet {

struct SetServoMessage : Message {
  std::vector<int> servo_indices;
  std::vector<float> target_radians;

  MESSAGE(SetServoMessage, servo_indices, target_radians);
};

}
 