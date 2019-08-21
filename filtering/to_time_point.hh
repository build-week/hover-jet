#include "infrastructure/time/timestamp.hh"
#include "third_party/experiments/estimation/time_point.hh"

namespace jet {
estimation::TimePoint to_time_point(const Timestamp& ts);
}  // namespace jet
