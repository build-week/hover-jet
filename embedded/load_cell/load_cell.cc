#include "embedded/load_cell/load_cell.hh"

#include <iostream>
#include <sstream>

namespace jet {

LoadCellReceiver::LoadCellReceiver(const std::string &path) : serial_port_path_(path) {
  const unsigned int SERIAL_BAUD_RATE = 38400;
  std::cout << "Attempting to connect on " << serial_port_path_ << std::endl;
  if (sp_get_port_by_name(serial_port_path_.c_str(), &serial_port_ptr_) == SP_OK) {
    sp_open(serial_port_ptr_, SP_MODE_READ_WRITE);
    sp_port_config *config = 0;
    sp_new_config(&config);
    sp_set_config_baudrate(config, SERIAL_BAUD_RATE);
    sp_set_config(serial_port_ptr_, config);
  } else {
    std::cerr << "Could not open serial port to load cells." << std::endl;
    std::abort();
  }
}

std::string LoadCellReceiver::read_bytes(const int buffer_byte_size) {
  const unsigned int SERIAL_TIMEOUT_MS = 200;
  // constexpr unsigned BUFFER_BYTE_SIZE = 1U;

  std::string byte_we_read(buffer_byte_size, ' ');
  sp_blocking_read(serial_port_ptr_, &byte_we_read[0], buffer_byte_size,
                   SERIAL_TIMEOUT_MS);

  return byte_we_read;
}

// Returns a vector containing the sensor identifier along with its raw reading.
// std::optional<ForceReading> LoadCellReceiver::parse(const std::string &tx) {
// }

std::optional<ForceReading> LoadCellReceiver::receive() {
  const std::string start_str = "SENSOR";

  std::string parsed_so_far;
  bool got_msg = false;
  while (!got_msg) {
    const char byte_we_read = read_bytes(1)[0];
    parsed_so_far += byte_we_read;

    const std::size_t loc = parsed_so_far.find(start_str);
    if (loc != std::string::npos) {
      got_msg = true;
      parsed_so_far = "";
      break;
    }
  }

  const char sensor_id_byte = read_bytes(1)[0];
  const uint8_t sensor_id = sensor_id_byte - static_cast<uint8_t>('0');
  std::cout << "sensor_id: " << static_cast<int>(sensor_id) << std::endl;
  if (sensor_id < 0 || sensor_id > 5) {
    // Invalid
    std::cout << "Failed to get valid sensor id byte" << std::endl;
    return {};
  }

  // Comma
  const char comma_byte = read_bytes(1)[0];
  if (comma_byte != ',') {
    std::cout << "Expected comma" << std::endl;
    return {};
  }

  const std::string float_4_bytes = read_bytes(4);

  float load_cell_value = 0.0;
  {
    *((uint8_t *)(&load_cell_value) + 3) = float_4_bytes[3];
    *((uint8_t *)(&load_cell_value) + 2) = float_4_bytes[2];
    *((uint8_t *)(&load_cell_value) + 1) = float_4_bytes[1];
    *((uint8_t *)(&load_cell_value) + 0) = float_4_bytes[0];
  }

  const char terminator = read_bytes(1)[0];
  if (terminator != ';') {
    std::cout << "Failed to get semicolon" << std::endl;
    return {};
  }

  std::cout << "Got: " << static_cast<int>(sensor_id) << ": " << load_cell_value
            << std::endl;
  return ForceReading({.id = sensor_id, .value = load_cell_value});
}

}  // namespace jet
