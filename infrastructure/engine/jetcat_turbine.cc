#include "jetcat_turbine.hh"

#include <algorithm>
#include <sstream>

#include <iostream>
//%deps(paho-mqttpp3)
//%deps(jet_serial)

namespace jet {

JetCatTurbine::JetCatTurbine() {
  std::string port = "/dev/ttyUSB0";
  serial_port_ =
      std::make_unique<serial::Serial>(port, 9600, serial::Timeout::simpleTimeout(200));
}

void JetCatTurbine::tokenize(const std::string& str,
                             std::vector<std::string>& tokens,
                             char delim = ',') const {
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    tokens.push_back(token);
  }
}

bool JetCatTurbine::handle_command_response(const std::string& command,
                                            const std::string& response) const {
  return command == response;
}

void JetCatTurbine::empty_the_buffer() const {
  size_t available_bytes = serial_port_->available();
  serial_port_->read(available_bytes);
}

bool JetCatTurbine::start_engine() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,1\r";  // Ex: "1,TCO,1\r"
  serial_port_->write(command_stream.str());
  std::string handshake_resp = serial_port_->read(command_stream.str().size());
  return handle_command_response(command_stream.str(), handshake_resp);
}

bool JetCatTurbine::stop_engine() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,0\r";  // Ex: "1,TCO,0\r"
  serial_port_->write(command_stream.str());
  std::string handshake_resp = serial_port_->read(command_stream.str().size());
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

bool JetCatTurbine::set_turbine_rpm(uint32_t target_rpm) const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WRP," << target_rpm << "\r";

  serial_port_->write(command_stream.str());
  std::string handshake_resp = serial_port_->read(command_stream.str().size());

  std::ostringstream expected_response_stream;
  expected_response_stream << command_stream.str() << turbine_slave_address_
                           << ",HS,OK\r";  // Ex: "1,WRP,10000\r1,HS,OK\r"

  if (!handle_command_response(expected_response_stream.str(), handshake_resp)) {
    return false;
  }

  return true;
}

std::optional<JetCat::SystemStatus> JetCatTurbine::get_system_status() const {
  empty_the_buffer();
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RSS,1\r";
  serial_port_->write(command_stream.str());
  std::string handshake_resp = serial_port_->read(command_stream.str().size());
  std::string data = serial_port_->read(100);

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
  serial_port_->write(command_stream.str());

  std::string handshake_resp = serial_port_->read(command_stream.str().size());
  std::string data = serial_port_->read(100);

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
  serial_port_->write(command_stream.str());

  std::string handshake_resp = serial_port_->read(command_stream.str().size());
  std::string data = serial_port_->read(100);

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
  serial_port_->write(command_stream.str());

  std::string handshake_resp = serial_port_->read(command_stream.str().size());
  std::string data = serial_port_->read(100);

  std::vector<std::string> tokens;
  tokenize(data, tokens, ',');

  if (tokens.size() != 9) {
    return {};
  }

  JetCat::FuelInfo info;
  info.actual_fuel_flow = std::stoi(tokens[3]);
  info.rest_volume_in_tank = std::stoi(tokens[4]);
  info.set_rpm = std::stoi(tokens[5]);
  info.actual_battery_voltage = std::stoi(tokens[6]);
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
