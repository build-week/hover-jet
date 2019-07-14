
#pragma once

#include "fmutex.hh"

namespace jet {

namespace {
const uint32_t message_size = 1000000 + 1024;
}

struct SharedStructMemoryRegion {
  Mutex region_futex;
  uint64_t message_len{0};
  char message[message_size];
};

}  // namespace jet
