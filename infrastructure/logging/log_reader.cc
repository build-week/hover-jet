#include "log_reader.hh"

#include "infrastructure/logging/logged_message_leader.hh"

#include <sys/stat.h>
#include <sys/types.h>

#include <stdint.h>

namespace jet {

std::string generate_log_file_path(const std::string& log_path, const std::string& channel_name, uint32_t file_number)
{
  return log_path + "/" + channel_name + "/" + std::to_string(file_number) + ".logfile";
}

bool is_directory(const std::string& path) {
  struct stat info;
  if (stat(path.data(), &info) == -1) {
    std::cout << "Directory " << path << " does not exist." << std::endl;
    return false;
  } else if (info.st_mode & S_IFDIR) {
    std::cout << path << " is a directory." << std::endl;
    return true;
  } else {
    std::cout << path << "is not a directory." << std::endl;
    return false;
  }
}

bool create_directory(const std::string& path) {
  std::cout << "Creating directory " << path << std::endl;
  const int error_code = mkdir(path.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (error_code == -1)
  {
    std::cout << "Error creating directory." << std::endl;
    return false;
  }
  return true;
}

LogReader::LogReader(const std::string& log_path, const std::vector<std::string>& channel_names) : log_path_(log_path) {
  if (!is_directory(log_path)) {
    throw std::runtime_error("Log path does not appear to exist: " + log_path);
  }

  if (!read_metadata(log_path, channels_in_log_)) {
    throw std::runtime_error("Couldn't open metadata file for log: " + log_path);
  }

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
    file.read((char*) channel_id, 4);
    file.read((char*) message_length, 4);
    std::string message_data(message_length, ' ');
    file.read(&message_data[0], message_length);
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
    file.read((char*) &channel_id, 4);
    file.read((char*) &message_length, 4);
    message_data.resize(message_length, ' ');
    file.read(&message_data[0], message_length);
    if (!file) {
      std::cout << "Failed to read from file for channel: " << channel_name << ". Reached end of file." <<  std::endl;
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
