#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

enum class IgnitionCommand { START, STOP };

struct TurbineIgnitionCommandMessage : Message {
  IgnitionCommand command;

  MESSAGE(TurbineIgnitionCommandMessage, command);
};

}  // namespace jet
