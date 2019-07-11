#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include <vector>

namespace jet {

struct ServoPwmSettingMessage : Message {
  std::vector<int> servo_indices;
  std::vector<int> pwm_set_points;

  MESSAGE(ServoPwmSettingMessage, servo_indices, pwm_set_points);
};

}
 