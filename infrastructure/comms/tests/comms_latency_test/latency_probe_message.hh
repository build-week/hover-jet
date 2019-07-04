#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

// 
struct LatencyProbeMessage : Message {
  std::string content;

  MESSAGE(LatencyProbeMessage, content);
};

} //  namespace jet
