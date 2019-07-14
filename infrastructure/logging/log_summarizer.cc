//%bin(log_summarizer)

//%deps(yaml-cpp)

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
  // for (std::size_t i = 0; i < channels.size(); ++i) {
    jet::ImuMessage message;
    if (log_reader.read_next_message("imu_2", message)) {
      channel_stats[0].first_message_received = message.header.timestamp_ns;
      channel_stats[0].num_messages_received++;
    }
    while(log_reader.read_next_message("imu_2", message)) {
      std::cout << "imu_2 Timestamp " << message.header.timestamp_ns << std::endl;
      channel_stats[0].last_message_received = message.header.timestamp_ns;
      channel_stats[0].num_messages_received++;
    }
  // }

  // Do last calculations
  // for (ChannelStats& stats : channel_stats) {
  //   stats.frequency = static_cast<double>(stats.num_messages_received)/(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(stats.last_message_received - stats.first_message_received))).count();
  // }

  // std::cout << "Channel\t\tFirst Timestamp\t\tLast Timestamp\t\tNum Messages\t\tFrequency" << std::endl;
  // // Print out results
  // for (std::size_t i = 0; i < channels.size(); ++i) {
  //   std::cout << channels[i] << "\t\t" << 
  //                channel_stats[i].first_message_received << "\t\t" << 
  //                channel_stats[i].last_message_received << "\t\t" << 
  //                channel_stats[i].num_messages_received << "\t\t" << 
  //                channel_stats[i].frequency << std::endl; 
                   
  // }

  
  return 0;
}
