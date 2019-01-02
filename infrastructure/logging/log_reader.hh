#pragma once

#include "infrastructure/comms/schemas/message.hh"

#include <fstream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace jet {

struct ChannelState {
  std::ifstream current_file;
  uint32_t channel_id{0};
  uint32_t current_file_number{0};
};

class LogReader {
 public:
  // Construct a log reader.
  // @param 
  // @param 
  LogReader(const std::string& log_path, const std::vector<std::string>& channel_names);

  // Destroy this LogReader
  ~LogReader();

  // Reads a string of bytes from the log file for the channel specified.
  // @param channel_name     The channel to read next message from.
  // @param message          A message reference to be populated with message content.
  bool read_next_message(const std::string& channel_name, Message& message);
  bool read_next_message_raw(const std::string& channel_name, std::string& message_data);

  std::vector<std::string> get_available_channels() {
    return channels_in_log_;
  }

 private:
  bool open_file(const std::string& file_path, std::ifstream& file);
  bool read_metadata(const std::string& log_path, std::vector<std::string>& channel_names);

  std::string log_path_;
  std::unordered_map<std::string, ChannelState> channels_;
  std::vector<std::string> channels_in_log_;
};

}  // namespace jet
