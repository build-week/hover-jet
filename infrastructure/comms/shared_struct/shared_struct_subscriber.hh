
#pragma once

#include "infrastructure/comms/subscriber.hh"
// #include <cstdint>

// #include <iostream>
// #include <optional>
// #include <string>

#include "infrastructure/time/timestamp.hh"
#include "infrastructure/comms/schemas/message.hh"

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>

#include <sys/mman.h>
#include <assert.h>

#include <iostream>
#include <cstdio>

#include <unistd.h>
#include <sys/types.h>

#include <cstring>

#include <pthread.h>


#include "fmutex.hh"

//%deps(rt)

#pragma once

namespace jet {

class SharedStructSubscriber : Subscriber {
public:
  SharedStructSubscriber(const std::string& object_name);
  ~SharedStructSubscriber();
  bool read(Message& message, const Duration& timeout = 0);
  bool read_raw(std::string& data, const Duration& timeout = 0);
  bool read_latest(Message& message, const Duration& timeout = 0);

private:
  struct SharedStructMemoryRegion {
    Mutex region_futex;
    uint64_t message_len{0};
    char message[1000000 + 1024];
  };

  SharedStructMemoryRegion * shmem_region_ptr_;
  int fd_;
  uint64_t most_recent_sequence_number_ = std::numeric_limits<uint64_t>::max();
  Timestamp most_recent_timestamp_{0};
};

}  // namespace jet

#include "shared_struct_subscriber.inl"
