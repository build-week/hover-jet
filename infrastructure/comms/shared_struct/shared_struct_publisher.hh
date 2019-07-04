
#pragma once

#include "infrastructure/comms/publisher.hh"
// #include <cstdint>

// #include <iostream>
// #include <optional>
// #include <string>

#include "infrastructure/time/time_utils.hh"

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


namespace jet {

// template <typename MessageType>
class SharedStructPublisher : Publisher {
public:
  SharedStructPublisher(const std::string& object_name);
  ~SharedStructPublisher();
  void publish(Message& message);
  void publish_raw(const std::string& data);

private:
  struct SharedStructMemoryRegion {
    Mutex region_futex;
    uint64_t message_len{0};
    // MessageType message;
    char message[1000000 + 1024];
  };

  // uint64_t sequence_number_{0};
  SharedStructMemoryRegion * shmem_region_ptr_;
  int fd_;
};

}  // namespace jet

#include "shared_struct_publisher.inl"
