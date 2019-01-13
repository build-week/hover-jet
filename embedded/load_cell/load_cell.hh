// %deps(serialport)
#include <memory>
#include <optional>
#include <string>

#include <libserialport.h>

namespace jet {

struct ForceReading {
  size_t id;
  float value;
};

class LoadCellReceiver {
 public:
  LoadCellReceiver(const std::string& path);
  std::optional<ForceReading> receive();

 private:
  const std::string serial_port_path_;
  sp_port* serial_port_ptr_ = 0;

  std::string read_bytes(const int buffer_byte_size);
};

}  // namespace jet
