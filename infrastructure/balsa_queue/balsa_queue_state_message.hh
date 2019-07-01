#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <string>

namespace jet {

struct BQStateMessage : Message {
  std::string bq_instance_name;

  MESSAGE(BQStateMessage, bq_instance_name);
};

}  //  namespace jet
