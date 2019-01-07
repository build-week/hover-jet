#pragma once

#include <sys/stat.h>
#include <sys/types.h>

namespace jet {

inline std::string generate_log_file_path(const std::string& log_path, const std::string& channel_name, uint32_t file_number)
{
  return log_path + "/" + channel_name + "/" + std::to_string(file_number) + ".logfile";
}

inline bool is_directory(const std::string& directory_path) {
  struct stat stat_info;
  if (stat(directory_path.data(), &stat_info) == -1) {
    return false;
  } else if (stat_info.st_mode & S_IFDIR) {
    return true;
  } else {
    return false;
  }
}

inline bool create_directory(const std::string& path) {
  const int error_code = mkdir(path.data(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (error_code == -1)
  {
    return false;
  }
  return true;
}

}
