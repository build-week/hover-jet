#include "jetcat_turbine.hh"

#include <algorithm>
#include <sstream>

#include <iostream>

//%deps(paho-mqttpp3)
//%deps(serialport)

namespace jet {

namespace {
  const unsigned int SERIAL_BAUD_RATE  = 9600;
  const unsigned int SERIAL_TIMEOUT_MS = 200;
}

JetCatTurbine::JetCatTurbine(const std::string& serial_port_path)
    : serial_port_path_(serial_port_path) {
  if (sp_get_port_by_name("/dev/ttySAC2", &serial_port_ptr_) == SP_OK)
  {
    sp_open(serial_port_ptr_, SP_MODE_READ_WRITE);
    sp_port_config *config = 0;
    sp_new_config(&config);
    sp_set_config_baudrate(config, SERIAL_BAUD_RATE);
    sp_set_parity(serial_port_ptr_, SP_PARITY_NONE);
    sp_set_parity(serial_port_ptr_, SP_PARITY_NONE);
    sp_set_stopbits(serial_port_ptr_, 1);
    sp_set_bits(serial_port_ptr_, 8);
    sp_set_config(serial_port_ptr_, config);
  } else {
    std::cerr << "Could not open serial port to turbine." << std::endl;
    exit(1);
  }
}

void JetCatTurbine::tokenize(const std::string& input_string,
                             std::vector<std::string>& tokens,
                             char delimiter = ',') const {
  std::stringstream ss(input_string);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
}

bool JetCatTurbine::handle_command_response(const std::string& command,
                                            const std::string& response) const {
  return command == response;
}

void JetCatTurbine::empty_the_buffer() const {
  size_t available_bytes = sp_input_waiting(serial_port_ptr_);
  std::string dev_null_buffer(available_bytes, ' ');
  sp_blocking_read(serial_port_ptr_, &dev_null_buffer[0], available_bytes, 0);
}

bool JetCatTurbine::start_engine() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,1\r";  // Ex: "1,TCO,1\r"

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  return handle_command_response(command_stream.str(), handshake_resp);
}

bool JetCatTurbine::stop_engine() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,0\r";  // Ex: "1,TCO,0\r"

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  return handle_command_response(command_stream.str(), handshake_resp);
}

void JetCatTurbine::set_serial_control_mode(bool on) const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WSM," << on
                 << "\r";  // Ex: "1,WSM,0\r" or "1,WSM,1\r"

  std::ostringstream expected_response_stream;
  expected_response_stream
      << command_stream.str() << turbine_slave_address_
      << ",HS,OK\r";  // Ex: "1,WSM,1\r1,HS,OK\r" or "1,WSM,0\r1,HS,OK\r"
}

bool JetCatTurbine::set_target_rpm(uint32_t target_rpm) const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WRP," << target_rpm << "\r";

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::ostringstream expected_response_stream;
  expected_response_stream << command_stream.str() << turbine_slave_address_
                           << ",HS,OK\r";  // Ex: "1,WRP,10000\r1,HS,OK\r"

  if (!handle_command_response(expected_response_stream.str(), handshake_resp)) {
    return false;
  }

  return true;
}

bool JetCatTurbine::set_thrust_percent(uint16_t thrust_percent) const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WPE," << thrust_percent << "\r";

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::ostringstream expected_response_stream;
  expected_response_stream << command_stream.str() << turbine_slave_address_
                           << ",HS,OK\r";  // Ex: "1,WPE,55\r1,HS,OK\r"

  if (!handle_command_response(expected_response_stream.str(), handshake_resp)) {
    return false;
  }

  return true;
}

std::optional<JetCat::SystemStatus> JetCatTurbine::get_system_status() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RSS,1\r";

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::string data(100, ' ');
  sp_blocking_read(serial_port_ptr_, &data[0], 100, SERIAL_TIMEOUT_MS);


  std::vector<std::string> tokens;
  tokenize(data, tokens, ',');
  if (tokens.size() != 9) {
    return {};
  }

  JetCat::SystemStatus status;
  status.off_condition = JetCat::OffCondition(std::stoi(tokens[3]));
  status.actual_flight_speed = std::stoi(tokens[4]);
  status.proportional_part_of_speed_regulator = std::stoi(tokens[5]);
  status.AD_value_of_AirSpeed_input = std::stoi(tokens[6]);
  status.AD_Zero_value_of_AirSpeed_input = std::stoi(tokens[7]);

  return status;
}

std::optional<JetCat::LiveValues> JetCatTurbine::get_live_values() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RAC,1\r";


  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::string data(100, ' ');
  sp_blocking_read(serial_port_ptr_, &data[0], 100, SERIAL_TIMEOUT_MS);

  std::vector<std::string> tokens;
  tokenize(data, tokens, ',');

  if (tokens.size() != 9) {
    return {};
  }

  JetCat::LiveValues values;
  values.turbine_rpm = std::stoi(tokens[3]);
  values.exhaust_gas_temperature_c = std::stoi(tokens[4]);
  values.pump_voltage = std::stof(tokens[5]);
  values.turbine_state = JetCat::State(std::stoi(tokens[6]));
  values.throttle_position_percent = std::stoi(tokens[7]);

  return values;
}

std::optional<JetCat::TurbineInfo> JetCatTurbine::get_turbine_info() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RTY,1\r";

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::string data(100, ' ');
  sp_blocking_read(serial_port_ptr_, &data[0], 100, SERIAL_TIMEOUT_MS);

  std::vector<std::string> tokens;
  tokenize(data, tokens, ',');

  if (tokens.size() != 9) {
    return {};
  }

  JetCat::TurbineInfo info;
  info.firmware_version_type = tokens[3];
  info.version_number = tokens[4];
  info.last_run_time = std::stoi(tokens[5]);
  info.total_operation_time = std::stoi(tokens[6]);
  info.serial_number = std::stoi(tokens[7]);
  info.turbine_type = tokens[8];

  return info;
}

std::optional<JetCat::FuelInfo> JetCatTurbine::get_fuel_info() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RFI,1\r";

  sp_blocking_write(serial_port_ptr_, &(command_stream.str()[0]), command_stream.str().size(), SERIAL_TIMEOUT_MS);
  std::string handshake_resp(command_stream.str().size(), ' ');
  sp_blocking_read(serial_port_ptr_, &handshake_resp[0], command_stream.str().size(), SERIAL_TIMEOUT_MS);

  std::string data(100, ' ');
  sp_blocking_read(serial_port_ptr_, &data[0], 100, SERIAL_TIMEOUT_MS);

  std::vector<std::string> tokens;
  tokenize(data, tokens, ',');

  if (tokens.size() != 9) {
    return {};
  }

  JetCat::FuelInfo info;
  info.actual_fuel_flow = std::stoi(tokens[3]);
  info.rest_volume_in_tank = std::stoi(tokens[4]);
  info.set_rpm = std::stoi(tokens[5]);
  info.actual_battery_voltage = std::stof(tokens[6]);
  info.last_run_time_s = std::stoi(tokens[7]);

  return info;
}

std::optional<uint32_t> JetCatTurbine::get_RPM() const {
  std::optional<JetCat::LiveValues> values = get_live_values();
  if (!values) {
    return {};
  }
  return values->turbine_rpm;
}

std::optional<uint32_t> JetCatTurbine::get_EGT() const {
  std::optional<JetCat::LiveValues> values = get_live_values();
  if (!values) {
    return {};
  }
  return values->exhaust_gas_temperature_c;
}

std::optional<float> JetCatTurbine::get_pump_voltage() const {
  std::optional<JetCat::LiveValues> values = get_live_values();
  if (!values) {
    return {};
  }
  return values->pump_voltage;
}

std::optional<JetCat::State> JetCatTurbine::get_turbine_state() const {
  std::optional<JetCat::LiveValues> values = get_live_values();
  if (!values) {
    return {};
  }
  return values->turbine_state;
}

std::optional<uint32_t> JetCatTurbine::get_throttle_position_percent() const {
  std::optional<JetCat::LiveValues> values = get_live_values();
  if (!values) {
    return {};
  }
  return values->throttle_position_percent;
}

}  // namespace jet
