#pragma once

#include "message.hh"

#include <string>

namespace jet {

// A demo message to be used as an example of how messages are defined
struct DemoMessage : Message {
  std::string content;

  MESSAGE(DemoMessage, content);
};

} //  namespace jet
