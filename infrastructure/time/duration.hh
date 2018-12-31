#pragma once

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

class Duration {
 public:
  Duration() = default;
  Duration(const uint64_t duration_ns) {
    duration_ns_ = duration_ns;
  };
  operator uint64_t() const {
    return duration_ns_;
  }

 private:
  uint64_t duration_ns_{0};

 public:
  SERIALIZABLE_STRUCTURE(Duration, duration_ns_);
};

}  // namespace jet
