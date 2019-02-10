#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {


struct JoystickCommandMessage : Message {
  double command;

  MESSAGE(JoystickCommandMessage, command);
};

}  // namespace jet
