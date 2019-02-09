#pragma once

#include "infrastructure/comms/schemas/message.hh"


#include <string>

namespace jet {

// A message for balsaqs to publish containing:
// 
struct GoNoGoMessage : Message {
  std::string bq_name = "";
  bool ready = false;
  std::string status_message = "";

  MESSAGE(GoNoGoMessage,
          bq_name,
          ready,
          status_message
  );
};

} //  namespace jet
