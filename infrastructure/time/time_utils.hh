#pragma once

#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>

namespace jet::time {

inline Timestamp get_current_time() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

}
