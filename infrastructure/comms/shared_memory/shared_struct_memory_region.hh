
#pragma once

#include "fmutex.hh"

namespace jet {

struct SharedStructMemoryRegion {
  Mutex region_futex;
  uint64_t message_len{0};
  char message[1000000 + 1024];
};

}  // namespace jet
