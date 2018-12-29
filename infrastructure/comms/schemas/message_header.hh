#pragma once

#include "third_party/nop/structure.h"

struct MessageHeader {
  uint64_t sequence_number{0};
  uint64_t timestamp_ns{0};

  NOP_STRUCTURE(MessageHeader, sequence_number, timestamp_ns);
};
