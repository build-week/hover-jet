//%bin(log_summarizer)

//%deps(yaml-cpp)
#include <iostream>
#include <iomanip>
#include "infrastructure/logging/log_reader.hh"
#include "infrastructure/config/config.hh"
#include "infrastructure/time/timestamp.hh"
#include "embedded/imu_driver/imu_message.hh"


struct ChannelStats{
  jet::Timestamp first_message_received;
  jet::Timestamp last_message_received;
  int num_messages_received;
  double frequency;
};

std::optional<std::string> parse_commandline_arguments(int argc, char* argv[]) {
  if (argc > 1) {
    return argv[1];
  }
  return {};
}

int main(int argc, char* argv[]) {
  const std::optional<std::string> log_path = parse_commandline_arguments(argc, argv);
  Config log_config;
  if (!log_path.has_value()) {
    std::cout << "Please enter a log path" << std::endl;
    return 0;
  }

  jet::LogReader log_reader(*log_path, {});
  const std::vector<std::string> channels = log_reader.get_available_channels();
  std::vector<ChannelStats> channel_stats(channels.size());

  // Go through all the messages
  for (std::size_t i = 0; i < channels.size(); ++i) {
    jet::Message message = {};
    if (log_reader.read_next_message(channels[i], message)) {
      channel_stats[i].first_message_received = message.header.timestamp_ns;
      channel_stats[i].num_messages_received++;
    }
    while(log_reader.read_next_message(channels[i], message)) {
      channel_stats[i].last_message_received = message.header.timestamp_ns;
      channel_stats[i].num_messages_received++;
    }
    channel_stats[i].frequency = static_cast<double>(channel_stats[i].num_messages_received)/(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(channel_stats[i].last_message_received - channel_stats[i].first_message_received))).count();
  }

  // Print out results;
  const std::string reset("\033[0m");
  const std::string red_bold("\033[31;1m");
  std::cout << red_bold << std::setw(30) << "Channel" << std::setw(25) << "First Timestamp" << std::setw(25) << "Last Timestamp" << std::setw(15) << "Num Messages" << std::setw(15) << "Frequency (Hz)" << std::endl;
  std::cout << reset;
  for (std::size_t i = 0; i < channels.size(); ++i) {
    std::cout << std::setw(30) << channels[i] << 
                 std::setw(25) << std::fixed << std::setprecision(9) << channel_stats[i].first_message_received/static_cast<double>(1e9) << 
                 std::setw(25) << std::fixed << std::setprecision(9) << channel_stats[i].last_message_received/static_cast<double>(1e9) << 
                 std::setw(15) << channel_stats[i].num_messages_received << 
                 std::setw(15) << std::fixed << std::setprecision(3) << channel_stats[i].frequency << std::endl;
  }

  
  return 0;
}
