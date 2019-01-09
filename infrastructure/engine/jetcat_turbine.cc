#include "jetcat_turbine.hh"

#include <sstream>

//%deps(paho-mqttpp3)
//%deps(jet_serial)

namespace jet {

JetCatTurbine::JetCatTurbine() {
  std::string port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OGVB-if00-port0";
	serial_port_ = std::make_unique<serial::Serial>(port, 9600, serial::Timeout::simpleTimeout(1000));
}

void JetCatTurbine::start_engine() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,1\r"; // Ex: "1,TCO,1\r"
}

void JetCatTurbine::stop_engine() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",TCO,0\r"; // Ex: "1,TCO,0\r"
}

void JetCatTurbine::set_serial_control_mode(bool on) const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WSM," << on << "\r"; // Ex: "1,WSM,0\r" or "1,WSM,1\r"

  std::ostringstream expected_response_stream;
  expected_response_stream << command_stream.str() << turbine_slave_address_ << ",HS,OK\r"; // Ex: "1,WSM,1\r1,HS,OK\r" or "1,WSM,0\r1,HS,OK\r"
}

void JetCatTurbine::set_turbine_rpm(uint32_t target_rpm) const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",WRP," << target_rpm << "\r";

  std::ostringstream expected_response_stream;
  expected_response_stream << command_stream.str() << turbine_slave_address_ << ",HS,OK\r"; // Ex: "1,WRP,10000\r1,HS,OK\r"
}

JetCat::SystemStatus JetCatTurbine::get_system_status() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RSS,1\r";

  // std::ostringstream response_stream;
}

JetCat::LiveValues JetCatTurbine::get_live_values() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RAC,1\r";

}

JetCat::TurbineInfo JetCatTurbine::get_turbine_info() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RTY,1\r";

}

JetCat::FuelInfo JetCatTurbine::get_fuel_info() const {
  std::ostringstream command_stream;
  command_stream << turbine_slave_address_ << ",RFI,1\r";

}

uint32_t JetCatTurbine::get_RPM() const {
  JetCat::LiveValues values = get_live_values();
  return values.turbine_rpm;
}

uint32_t JetCatTurbine::get_EGT() const {
  JetCat::LiveValues values = get_live_values();
  return values.exhaust_gas_temperature_c;
}

float JetCatTurbine::get_pump_voltage() const {
  JetCat::LiveValues values = get_live_values();
  return values.pump_voltage;
}

JetCat::State JetCatTurbine::get_turbine_state() const {
  JetCat::LiveValues values = get_live_values();
  return values.turbine_state;
}

uint32_t JetCatTurbine::get_throttle_position_percent() const {
  JetCat::LiveValues values = get_live_values();
  return values.throttle_position_percent;
}

}  // namespace jet
