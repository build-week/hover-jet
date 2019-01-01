#pragma once

#include <fstream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace jet {

struct ChannelState {
  std::ofstream current_file;
  uint32_t channel_id{0};
  uint32_t current_file_number{0};
  uint64_t file_size_bytes{0};
  uint64_t last_flush_bytes{0};
};

class LogWriter {
 public:
  // Construct a log writer.
  // @param log_path       The path to the log. Ex: "/logs/logname"
  // @param channel_names  A vector of all the channel names that the logger should expect to log.
  LogWriter(const std::string& log_path, const std::vector<std::string>& channel_names);

  // Destroy this LogWriter. Flushes and closes all open log files.
  ~LogWriter();

  // Writes a string of bytes to the log file for the channel specified.
  // @param channel_name     The channel this message should be logged under.
  // @param serialized_data  The raw data to log.
  bool write_message(const std::string& channel_name, const std::string& serialized_data);

 private:
  bool create_log_directory(const std::string& log_path);
  bool open_file(const std::string& file_path, std::ofstream& file);
  bool write_metadata(const std::string& log_path, const std::vector<std::string>& channel_names);

  std::string log_path_;
  std::unordered_map<std::string, ChannelState> channels_;
};

}  // namespace jet
