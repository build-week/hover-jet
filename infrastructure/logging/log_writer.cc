#include "log_writer.hh"
#include "log_utils.hh"

#include "infrastructure/logging/logged_message_leader.hh"

#include "infrastructure/comms/schemas/message.hh"

#include <stdint.h>

namespace jet {

namespace {
constexpr uint32_t BYTES_PER_FLUSH = 1000;
constexpr uint64_t MAX_FILE_SIZE = 1'000'000'000;
}

LogWriter::LogWriter(const std::string& log_path, const std::vector<std::string>& channel_names) : log_path_(log_path) {
  if (!create_log_directory(log_path)) {
    throw std::runtime_error("Couldn't create directory for log: " + log_path);
  }

  if (!write_metadata(log_path, channel_names)) {
    throw std::runtime_error("Couldn't open metadata file for log: " + log_path);
  }

  for (auto& channel_name : channel_names) {
    if (!create_log_directory(log_path + "/" + channel_name)) {
      throw std::runtime_error("Couldn't create directory: " + log_path + "/" + channel_name);
    }
    ChannelState channel;
    const std::string file_path = generate_log_file_path(log_path, channel_name, channel.current_file_number);
    if (!open_file(file_path, channel.current_file))
    {
      throw std::runtime_error("Couldn't open file: " + file_path);
    }
    channels_.emplace(channel_name, std::move(channel));
  }
}

LogWriter::~LogWriter() {
  for (auto& channel : channels_)
  {
    channel.second.current_file.flush();
    channel.second.current_file.close();
  }
}

bool LogWriter::write_message(const std::string& channel_name, const std::string& serialized_data) {
  ChannelState& channel_state = channels_[channel_name];
  if (channel_state.file_size_bytes > channel_state.last_flush_bytes + BYTES_PER_FLUSH) {
    channel_state.current_file.flush();
    channel_state.last_flush_bytes = channel_state.file_size_bytes;
  }

  // If the current log file is too large, close it and open a new one.
  if (channel_state.file_size_bytes > MAX_FILE_SIZE) {
    channel_state.current_file.flush();
    channel_state.current_file.close();
    channel_state.current_file_number += 1;
    channel_state.file_size_bytes = 0;
    channel_state.last_flush_bytes = 0;
    std::string new_file_path = generate_log_file_path(log_path_, channel_name, channel_state.current_file_number);
    if (!open_file(new_file_path, channel_state.current_file)) {
      throw std::runtime_error("Couldn't open file: " + new_file_path);
    }
  } 

  // Write the channel ID and message length to the log file.
  uint32_t serialized_data_length = serialized_data.size();
  channel_state.current_file.write((char*) &channel_state.channel_id, sizeof(uint32_t));
  channel_state.current_file.write((char*) &serialized_data_length, sizeof(uint32_t));

  // Write the message itself to the log file.
  channel_state.current_file.write(serialized_data.data(), serialized_data.size());

  // Update the byte counter for this log file.
  channel_state.file_size_bytes += serialized_data.size() + sizeof(uint32_t) + sizeof(uint32_t);
  return true;
}

bool LogWriter::create_log_directory(const std::string& log_path)
{
  if (is_directory(log_path))
  {
    return true;
  }
  return create_directory(log_path);
}

bool LogWriter::write_metadata(const std::string& log_path, const std::vector<std::string>& channel_names)
{
  static const std::string newline = "\n";
  std::ofstream metadata_file;
  const std::string metadata_path = log_path + "/metadata.txt";
  if (!open_file(metadata_path, metadata_file))
  {
    return false;
  }
  for (auto channel_name : channel_names)
  {
    metadata_file.write(channel_name.data(), channel_name.size());
    metadata_file.write(newline.data(), newline.size());
  }
  metadata_file.close();
  return true;
}

bool LogWriter::open_file(const std::string& file_path, std::ofstream& file) {
  try {
    file.open(file_path.data(), std::ofstream::out | std::ofstream::binary);
  } catch (const std::ios_base::failure& e) {
    std::cerr << "Could not open file " << file_path << " because " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace jet
