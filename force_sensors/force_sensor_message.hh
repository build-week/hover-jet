#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/time/timestamp.hh"

namespace jet {

struct ForceSensorMessage : Message {
  int id;
  float value;
  Timestamp timestamp;

  MESSAGE(ForceSensorMessage, id, value, timestamp);
};

}  //  namespace jet
