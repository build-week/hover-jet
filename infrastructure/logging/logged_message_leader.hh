#pragma once

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

struct LoggedMessageLeader {
  uint32_t channel_id;
  uint32_t message_length;

  SERIALIZABLE_STRUCTURE(LoggedMessageLeader, channel_id, message_length);
};

} //  namespace jet
