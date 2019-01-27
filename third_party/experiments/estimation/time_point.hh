#pragma once

#include <chrono>

namespace estimation {

using TimePoint = std::chrono::system_clock::time_point;
using TimeDuration = TimePoint::duration;

constexpr TimeDuration to_duration(const double sec) {
  const auto dbl_dur = std::chrono::duration<double, std::ratio<1>>(sec);
  const auto real_dur = std::chrono::duration_cast<TimeDuration>(dbl_dur);
  return real_dur;
}

constexpr double to_seconds(const TimeDuration& t) {
  constexpr double SECONDS_PER_MICROSECOND = 1e-6;
  const double microseconds = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(t).count());
  return microseconds * SECONDS_PER_MICROSECOND;
}

}  // namespace estimation

namespace jcc {
inline estimation::TimePoint now() {
  return std::chrono::system_clock::now();
}

}  // namespace jcc
