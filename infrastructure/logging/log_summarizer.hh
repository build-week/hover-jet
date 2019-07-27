#pragma once

#include <iostream>
#include <iomanip>
#include <iterator>
#include "third_party/experiments/estimation/time_point.hh"
#include "infrastructure/time/timestamp.hh"
#include "infrastructure/logging/log_reader.hh"

namespace jet {

estimation::TimePoint to_time_point(const Timestamp& ts) {
  const auto epoch_offset = std::chrono::nanoseconds(uint64_t(ts));
  const estimation::TimePoint time_point = estimation::TimePoint{} + epoch_offset;
  return time_point;
}
  
struct ChannelStats{
  jet::Timestamp first_message_received;
  jet::Timestamp last_message_received;
  int num_messages_received;
  double frequency;
};

struct LogSummary {
  std::map<std::string, ChannelStats> channel_stats;
  
  LogSummary(const std::string log_path) {
    LogReader log_reader(log_path, {});
    const std::vector<std::string> channels = log_reader.get_available_channels();

    // Go through all the messages
    for (std::size_t i = 0; i < channels.size(); ++i) {
      Message message = {};
      const std::string channel_name = channels[i];
      if (log_reader.read_next_message(channel_name, message)) {
        channel_stats[channel_name].first_message_received = message.header.timestamp_ns;
        channel_stats[channel_name].num_messages_received++;
      }
      while(log_reader.read_next_message(channels[i], message)) {
        channel_stats[channel_name].last_message_received = message.header.timestamp_ns;
        channel_stats[channel_name].num_messages_received++;
      }
      const estimation::TimePoint start_time = to_time_point(channel_stats[channel_name].first_message_received);
      const estimation::TimePoint end_time = to_time_point(channel_stats[channel_name].last_message_received);
      channel_stats[channel_name].frequency = static_cast<double>(channel_stats[channel_name].num_messages_received)/estimation::to_seconds(end_time - start_time);
    }
  }

  void print() {
    const std::string reset("\033[0m");
    const std::string red_bold("\033[31;1m");
    std::cout << red_bold << std::setw(30) << "Channel" << std::setw(25) << "First Timestamp" << std::setw(25) << "Last Timestamp" << std::setw(17) << "Duration (s)" << std::setw(15) << "Num Messages" << std::setw(15) << "Frequency (Hz)" << std::endl;
    std::cout << reset;
    for(auto it = channel_stats.begin(); it != channel_stats.end(); it++) {
      const auto duration = (it->second.last_message_received - it->second.first_message_received)/static_cast<double>(1e9); 
      std::cout << std::setw(30) << it->first << 
                   std::setw(25) << std::fixed << std::setprecision(9) << it->second.first_message_received/static_cast<double>(1e9) << 
                   std::setw(25) << std::fixed << std::setprecision(9) << it->second.last_message_received/static_cast<double>(1e9) << 
                   std::setw(17) << std::fixed << std::setprecision(3) << duration << 
                   std::setw(15) << it->second.num_messages_received << 
                   std::setw(15) << std::fixed << std::setprecision(3) << it->second.frequency << std::endl;
    }
  }
};

}  // namespace jet
