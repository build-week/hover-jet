#pragma once

#include <stdint.h>  // uint8_t etc

namespace jet {

class Timestamp {
 public:
  Timestamp(const uint64_t time) {
    timestamp_ = time;
  };
  operator uint64_t() const {
    return timestamp_;
  }

 private:
  uint64_t timestamp_;
};

}  // namespace jet
