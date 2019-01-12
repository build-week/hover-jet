#pragma once

#include "infrastructure/comms/schemas/message.hh"

namespace jet {

struct SetServoMessage : Message {
  float target_angle; //TODO units

  MESSAGE(SetServoMessage, target_angle);
};

}
 