#pragma once

#include <chrono>

#include "infrastructure/comms/serialization/serialization_macros.hh"

namespace jet {

class Timestamp {
 public:
  // So that everything uses the same block
  using Clock = std::chrono::system_clock;
  using ClockPoint = Clock::time_point;
  using TimestampRatio = std::chrono::nanoseconds;

  Timestamp() = default;
  Timestamp(const uint64_t nanoseconds_since_epoch) {
    timestamp_ = nanoseconds_since_epoch;
  };

  // Construct from a std::chrono timepoint, namely the `Clock` specified in this header
  Timestamp(const ClockPoint& time) {
    const auto time_nanos = std::chrono::duration_cast<TimestampRatio>(time.time_since_epoch());
    timestamp_ = time_nanos.count();
  }

  operator uint64_t() const {
    return timestamp_;
  }

  // Get the timepoint associated with this timestamp
  ClockPoint time_point() const {
    TimestampRatio nanoseconds(timestamp_);
    return ClockPoint{} + nanoseconds;
  }

  // Get the current time from the timestamp clock
  static Timestamp current_time() {
    return Timestamp(current_time_point());
  }

 private:
  // Get the current time from the timestamp clock as a time point
  static ClockPoint current_time_point() {
    return Clock::now();
  }

  uint64_t timestamp_{0};

 public:
  SERIALIZABLE_STRUCTURE(Timestamp, timestamp_);
};

// Shove the timestamp into an ostream
inline std::ostream& operator<<(std::ostream& out, const Timestamp& timestamp) {
  out << uint64_t(timestamp);
  return out;
}

}  // namespace jet
