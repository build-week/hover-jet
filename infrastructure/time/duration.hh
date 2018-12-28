#pragma once

#include <stdint.h>  // uint8_t etc

namespace jet {

class Duration {
 public:
  Duration(const uint64_t duration_ns) {
    duration_ns_ = duration_ns;
  };
  operator uint64_t() const {
    return duration_ns_;
  }

 private:
  uint64_t duration_ns_;
};

}  // namespace jet
