#pragma once

#include "infrastructure/comms/schemas/message.hh"


#include <string>

namespace jet {

// A message for balsaqs to publish containing:
// 
struct GoNoGoMessage : Message {
  std::string bq_name = "";
  bool ready_ = false;
  std::string status_message_ = "";
  // std::optional<std::string> status_message_ = "";

  MESSAGE(GoNoGoMessage,
          bq_name,
          ready_,
          status_message_
  );
};

} //  namespace jet
