#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

// A message for balsaqs to publish containing:
// 
struct GoNoGoMessage : Message {
  bool ready = false;
  std::string statusMessage = "";

  MESSAGE(GoNoGoMessage,
          ready,
          statusMessage
  );
};

} //  namespace jet
