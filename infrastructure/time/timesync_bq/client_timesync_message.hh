#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

#include <string>

#include <unistd.h>
#include <limits.h>

namespace jet {

// A message for timesync clients to publish containing:
// master message timestamp & seq
// client timestamp & seq
struct ClientTimesyncMessage : Message {
  Timestamp master_timestamp{0};
  uint64_t master_seq_number{0};
  char hostname[HOST_NAME_MAX];

  MESSAGE(ClientTimesyncMessage, 
        master_timestamp,
        master_seq_number,
        hostname);
};

} //  namespace jet
