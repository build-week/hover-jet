#include "log_reader.hh"
#include "log_utils.hh"

#include "infrastructure/logging/logged_message_leader.hh"

#include <stdint.h>

namespace jet {

LogReader::LogReader(const std::string& log_path, const std::vector<std::string>& channel_names) : log_path_(log_path) {
  if (!is_directory(log_path)) {
    throw std::runtime_error("Log path does not appear to exist: " + log_path);
  }

  if (!read_metadata(log_path, channels_in_log_)) {
    throw std::runtime_error("Couldn't open metadata file for log: " + log_path);
  }

  if (channel_names.empty()) {
    for (auto& channel_name : channels_in_log_) {
      ChannelState channel;
      if (!get_logfile(channel, channel_name)) {
        throw std::runtime_error("Couldn't open file for " + channel_name);
      }
      channels_.emplace(channel_name, std::move(channel));
    }
  }
  else {
    for (auto& channel_name : channel_names) {
      auto channel_it = std::find(channels_in_log_.begin(), channels_in_log_.end(), channel_name);
      if (channel_it != channels_in_log_.end()) {
        ChannelState channel;
        if (!get_logfile(channel, channel_name)) {
          throw std::runtime_error("Couldn't open file for " + channel_name);
        }
        channels_.emplace(channel_name, std::move(channel));
      }
      else {
        std::cerr << "Requested channel " << channel_name << " does not appear to exist in this log: " << log_path << std::endl;
      }
    }
  }
}

LogReader::~LogReader() {
  for (auto& channel : channels_) {
    channel.second.current_file.close();
  }
}

bool LogReader::get_logfile(ChannelState& channel, std::string channel_name) {
  if (channel.current_file.is_open()) {
    channel.current_file.close();
  }
  const std::string file_path = generate_log_file_path(log_path_, channel_name, channel.current_file_number);
  return open_file(file_path, channel.current_file);
 
}

bool LogReader::try_read_next_message(ChannelState& channel, Message& message) {
  auto& file = channel.current_file;

  uint32_t channel_id;
  uint32_t message_length;
  file.read((char*) &channel_id, sizeof(uint32_t));
  file.read((char*) &message_length, sizeof(uint32_t));

  if (!file.good()) {
    return false;
  }

  std::string message_data(message_length, ' ');
  file.read(&message_data[0], message_length);
  if (!file.good()) {
    return false;      
  }
  message.deserialize(message_data);
  return true;
}

bool LogReader::read_next_message(const std::string& channel_name, Message& message) {
  auto channel_it = channels_.find(channel_name);
  if (channel_it == channels_.end()) {
    return false;
  }
  // Try to read a message from the current log file
  if (try_read_next_message(channel_it->second, message)) {
    return true;
  }
  // Otherwise get the next logfile if it exists
  channel_it->second.current_file_number++;
  if (!get_logfile(channel_it->second, channel_name)) {
    return false;
  }
  // If we can't read from next logfile then give up
  return try_read_next_message(channel_it->second, message);
}

bool LogReader::try_read_next_message_raw(ChannelState& channel, std::string& message_data) {
  auto& file = channel.current_file;
  uint32_t channel_id;
  uint32_t message_length;
  file.read((char*) &channel_id, sizeof(uint32_t));
  file.read((char*) &message_length, sizeof(uint32_t));

  if (!file.good()) {
    return false;
  }

  message_data.resize(message_length, ' ');
  file.read(&message_data[0], message_length);

  return file.good();
}

bool LogReader::read_next_message_raw(const std::string& channel_name, std::string& message_data) {
  auto channel_it = channels_.find(channel_name);
  if (channel_it == channels_.end()) {
    return false;
  }
  // Try to read a message from the current log file
  if (try_read_next_message_raw(channel_it->second, message_data)) {
    return true;
  }
  // Otherwise get the next log file if it exists
  channel_it->second.current_file_number++;
  if (!get_logfile(channel_it->second, channel_name)) {
    return false;
  }
  // If we can't read from the next logfile, then give up
  return try_read_next_message_raw(channel_it->second, message_data);
}

bool LogReader::read_metadata(const std::string& log_path, std::vector<std::string>& channel_names) {
  const std::string metadata_path = log_path + "/metadata.txt";
  std::ifstream metadata_file;
  if (!open_file(metadata_path, metadata_file)) {
    return false;
  }

  std::string line;
  while (std::getline(metadata_file, line)) {
    channels_in_log_.emplace_back(line);
  }
  metadata_file.close();
  return true;
}

bool LogReader::open_file(const std::string& file_path, std::ifstream& file) {
  try {
    file.open(file_path.data(), std::ifstream::in | std::ifstream::binary);
  } catch (const std::ios_base::failure& e) {
    std::cerr << "Could not open file " << file_path << " because " << e.what() << std::endl;
    return false;
  }
  return file.good();
}

}  // namespace jet
