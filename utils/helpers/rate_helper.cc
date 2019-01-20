#include "rate_helper.hh"

#include <algorithm>
#include <iomanip>
#include <numeric>

namespace jet {
namespace utils {

RateHelper::RateHelper(const size_t &window_size) : msg_received_(0) {
  std::cout << "";
}

void RateHelper::update(const Timestamp &timestamp) {
  ++msg_received_;
  msg_timestamps_.push_back(std::chrono::nanoseconds(timestamp));

  std::cout << "Incoming Timestamp: " << msg_timestamps_.back().count() << std::endl;

  if (msg_timestamps_.size() >= 3U) {
    const auto stats = calculate_stats();
    std::cout << "Average Rate: " << std::setprecision(3) << stats.rate_hz << "hz" << std::endl;
    std::cout << "\t" << std::setprecision(5) << "min: " << stats.min_ts_delta << "s  "
              << "max: " << stats.max_ts_delta << "s  "
              << "std dev: " << stats.std_dev << std::endl;
  } else {
    std::cout << "Not enough messages have been received - Last msg timestamp: " << timestamp << std::endl;
  }
}

msg_stats RateHelper::calculate_stats() {
  msg_stats stats;

  std::vector<std::chrono::duration<double>> adjacent_differences;
  std::chrono::nanoseconds difference_sum = std::chrono::nanoseconds::zero();

  const auto find_adjacent_time_difference = [&adjacent_differences,
                                              &difference_sum](const std::vector<std::chrono::nanoseconds> &vec) {
    for (auto pos = vec.end() - 1; pos != vec.begin() + 1; --pos) {
      const auto &current_ts = *pos;
      const auto &prev_adj_ts = *(pos - 1);
      const auto duration_ns = (current_ts - prev_adj_ts);

      adjacent_differences.emplace_back(duration_ns);
      difference_sum += duration_ns;
    }
  };

  find_adjacent_time_difference(msg_timestamps_);
  const double avg_msg_transmit_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(difference_sum).count() / adjacent_differences.size();

  // Populate stats
  stats.rate_hz = 1.0 / avg_msg_transmit_s;

  const auto min_max_ts_delta = std::minmax_element(adjacent_differences.begin(), adjacent_differences.end());
  stats.min_ts_delta = std::chrono::duration_cast<std::chrono::duration<double>>(*min_max_ts_delta.first).count();
  stats.max_ts_delta = std::chrono::duration_cast<std::chrono::duration<double>>(*min_max_ts_delta.second).count();
  stats.std_dev = 1.0;

  return stats;
}

}  // namespace utils
}  // namespace jet
