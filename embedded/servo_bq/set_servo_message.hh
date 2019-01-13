#pragma once

#include "infrastructure/comms/schemas/message.hh"
// #include "infrastructure/comms/serialization/serialization_macros.hh"
#include <vector>

namespace jet {

// struct ServoCommand{
// 	int servo_index;
// 	float target_percentage;
// 	SERIALIZABLE_STRUCTURE(ServoCommand, servo_index, target_percentage);
// };

struct SetServoMessage : Message {
  std::vector<int> servo_indices;
  std::vector<float> target_percentages;

  MESSAGE(SetServoMessage, servo_indices, target_percentages);
};

}
 