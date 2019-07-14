
#pragma once

#include "infrastructure/comms/schemas/message.hh"
#include "infrastructure/comms/shared_memory/shared_struct_memory_region.hh"
#include "infrastructure/comms/subscriber.hh"
#include "infrastructure/time/timestamp.hh"

#include <assert.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>

//%deps(rt)

namespace jet {

class SharedStructSubscriber : public Subscriber {
 public:
  SharedStructSubscriber(const std::string& object_name);
  ~SharedStructSubscriber();
  bool read(Message& message, const Duration& timeout = 0);
  bool read_raw(std::string& data, const Duration& timeout = 0);
  bool read_latest(Message& message, const Duration& timeout = 0);

 private:
  SharedStructMemoryRegion* shmem_region_ptr_;
  int shmem_file_descriptor_;
  uint64_t most_recent_sequence_number_ = std::numeric_limits<uint64_t>::max();
  Timestamp most_recent_timestamp_{0};
};

}  // namespace jet

#include "shared_struct_subscriber.inl"
