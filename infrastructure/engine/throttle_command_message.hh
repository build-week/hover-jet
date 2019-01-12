#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

struct ThrottleCommandMessage : Message {
  uint16_t throttle_percent;

  MESSAGE(ThrottleCommandMessage, throttle_percent);
};

}  // namespace jet
