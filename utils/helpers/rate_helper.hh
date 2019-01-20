
#pragma once

#include "infrastructure/time/timestamp.hh"

#include <chrono>
#include <vector>

namespace jet {
namespace utils {

struct msg_stats {
  double rate_hz;
  double std_dev;
  double max_ts_delta;
  double min_ts_delta;
};

class RateHelper {
 public:
  RateHelper(const size_t &window_size);

  void update(const Timestamp &ts);
  double get_rate_hz() const;
  double get_rate_mean() const;
  double get_rate_std() const;

 private:
  msg_stats calculate_stats();

 private:
  // Number of messages that have been received.
  unsigned int msg_received_;

  // Collection of message receive times, may be useful for displaying statistics
  std::vector<std::chrono::nanoseconds> msg_timestamps_;
};

}  // namespace utils
}  // namespace jet
