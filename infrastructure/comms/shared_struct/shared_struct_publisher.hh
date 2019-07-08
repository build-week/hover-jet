
#pragma once

#include "infrastructure/comms/publisher.hh"
#include "infrastructure/time/time_utils.hh"
#include "infrastructure/comms/shared_memory/shared_struct_memory_region.hh"

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
#include <cstring>
#include <iostream>

//%deps(rt)

namespace jet {

class SharedStructPublisher : public Publisher {
 public:
  SharedStructPublisher(const std::string& object_name);
  ~SharedStructPublisher();
  void publish(Message& message);
  void publish_raw(const std::string& data);

 private:
  SharedStructMemoryRegion* shmem_region_ptr_;
  int shmem_file_descriptor_;
};

}  // namespace jet

#include "shared_struct_publisher.inl"
