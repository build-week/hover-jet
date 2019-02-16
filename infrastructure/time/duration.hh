#pragma once

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace {
  const uint64_t NANOSECONDS_PER_SECOND      = 1000000000;
  const uint64_t NANOSECONDS_PER_MICROSECOND = 1000000;
}

namespace jet {

class Duration {
 public:
  Duration() = default;
  Duration(const uint64_t duration_ns) {
    duration_ns_ = duration_ns;
  };
  static Duration from_seconds(const uint64_t seconds) {
    return Duration(seconds * NANOSECONDS_PER_SECOND);
  }
  static Duration from_nanoseconds(const uint64_t nanoseconds) {
    return Duration(nanoseconds);
  }
  static Duration from_microseconds(const uint64_t microseconds) {
    return Duration(microseconds * NANOSECONDS_PER_MICROSECOND);
  }
  operator uint64_t() const {
    return duration_ns_;
  }

 private:
  uint64_t duration_ns_{0};

 public:
  SERIALIZABLE_STRUCTURE(Duration, duration_ns_);
};

}  // namespace jet
