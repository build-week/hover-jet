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

  if (channel_names.empty())
  {
    for (auto& channel_name : channels_in_log_) {
      ChannelState channel;
      const std::string file_path = generate_log_file_path(log_path, channel_name, channel.current_file_number);
      if (!open_file(file_path, channel.current_file))
      {
        throw std::runtime_error("Couldn't open file: " + file_path);
      }
      channels_.emplace(channel_name, std::move(channel));
    }
  }
  else {
    for (auto& channel_name : channel_names) {
      auto channel_it = std::find(channels_in_log_.begin(), channels_in_log_.end(), channel_name);
      if (channel_it != channels_in_log_.end())
      {
        ChannelState channel;
        const std::string file_path = generate_log_file_path(log_path, channel_name, channel.current_file_number);
        if (!open_file(file_path, channel.current_file))
        {
          throw std::runtime_error("Couldn't open file: " + file_path);
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
  for (auto& channel : channels_)
  {
    channel.second.current_file.close();
  }
}

bool LogReader::read_next_message(const std::string& channel_name, Message& message) {
  auto channel_it = channels_.find(channel_name);
  if (channel_it != channels_.end()) {
    auto& file = channel_it->second.current_file;
    uint32_t channel_id;
    uint32_t message_length;
    file.read((char*) &channel_id, sizeof(uint32_t));
    file.read((char*) &message_length, sizeof(uint32_t));
    std::string message_data(message_length, ' ');
    file.read(&message_data[0], message_length);
    if (!file) {
      // TODO: Check to see if there's another log file after this one before assuming we've reached the end of the log.
      std::cerr << "Failed to read from file for channel: " << channel_name << ". Reached end of file." <<  std::endl;
      return false;
    }
    message.deserialize(message_data);
    return true;
  }
  return false;
}

bool LogReader::read_next_message_raw(const std::string& channel_name, std::string& message_data) {
  auto channel_it = channels_.find(channel_name);
  if (channel_it != channels_.end()) {
    auto& file = channel_it->second.current_file;
    uint32_t channel_id;
    uint32_t message_length;
    file.read((char*) &channel_id, sizeof(uint32_t));
    file.read((char*) &message_length, sizeof(uint32_t));
    message_data.resize(message_length, ' ');
    file.read(&message_data[0], message_length);
    if (!file) {
      // TODO: Check to see if there's another log file after this one before assuming we've reached the end of the log.
      std::cerr << "Failed to read from file for channel: " << channel_name << ". Reached end of file." <<  std::endl;
      return false;
    }
    return true;
  }
  return false;
}

bool LogReader::read_metadata(const std::string& log_path, std::vector<std::string>& channel_names) {
  const std::string metadata_path = log_path + "/metadata.txt";
  std::ifstream metadata_file;
  if (!open_file(metadata_path, metadata_file))
  {
    return false;
  }

  std::string line;
  while (std::getline(metadata_file, line))
  {
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
  return true;
}

}  // namespace jet
