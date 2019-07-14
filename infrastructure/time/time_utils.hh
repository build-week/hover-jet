#pragma once

#include "infrastructure/time/timestamp.hh"

#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>

namespace jet::time {

inline Timestamp get_current_time() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}


inline Timestamp msec_monotonic_to_vehicle_monotonic(long int msec_monotonic) {
  // Get the reported monotonic time in milliseconds
  const std::chrono::milliseconds cap_time_monotonic_msec(msec_monotonic);
  // Convert it to a timepoint
  const std::chrono::time_point<std::chrono::steady_clock> cap_time_monotonic(cap_time_monotonic_msec);

  // Get the current time in monotonic
  const auto cur_time_monotonic = std::chrono::steady_clock::now();
  // Get the current time in the vehicle wall clock
  const auto cur_time_vehicle = get_current_time().time_point();
  const auto cur_time_from_cap_time = cur_time_monotonic - cap_time_monotonic;
  const auto cap_time_vehicle = cur_time_vehicle - cur_time_from_cap_time;

  const auto wall_from_monotonic = (cur_time_vehicle.time_since_epoch()) - (cur_time_monotonic.time_since_epoch());

  return Timestamp(cap_time_vehicle);
}


}
