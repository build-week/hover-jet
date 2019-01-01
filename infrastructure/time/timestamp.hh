#pragma once

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

class Timestamp {
 public:
  Timestamp() = default;
  Timestamp(const uint64_t time) {
    timestamp_ = time;
  };
  operator uint64_t() const {
    return timestamp_;
  }

 private:
  uint64_t timestamp_{0};

 public:
  SERIALIZABLE_STRUCTURE(Timestamp, timestamp_);
};

}  // namespace jet
