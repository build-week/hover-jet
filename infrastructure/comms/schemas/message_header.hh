#pragma once

#include "infrastructure/comms/serialization/serialization_macros.hh"
#include "infrastructure/time/timestamp.hh"

namespace jet {

struct MessageHeader {
  uint64_t sequence_number{0};
  Timestamp timestamp_ns{0};

  SERIALIZABLE_STRUCTURE(MessageHeader, sequence_number, timestamp_ns);
};

}  //  namespace jet
