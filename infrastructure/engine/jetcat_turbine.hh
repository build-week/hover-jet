#pragma once

#include "infrastructure/engine/jetcat_schemas.hh"

#include <memory>
#include <optional>
#include <vector>

#include <libserialport.h>

namespace jet {

class JetCatTurbine {
 public:
  JetCatTurbine(const std::string& serial_port_path);
  bool start_engine() const;
  bool stop_engine() const;
  void set_serial_control_mode(bool on) const;
  bool set_target_rpm(uint32_t target_rpm) const;
  bool set_thrust_percent(uint16_t thrust_percent) const;

  std::optional<JetCat::SystemStatus> get_system_status() const;
  std::optional<JetCat::LiveValues> get_live_values() const;
  std::optional<JetCat::TurbineInfo> get_turbine_info() const;
  std::optional<JetCat::FuelInfo> get_fuel_info() const;

  std::optional<uint32_t> get_RPM() const;
  std::optional<uint32_t> get_EGT() const;
  std::optional<float> get_pump_voltage() const;
  std::optional<JetCat::State> get_turbine_state() const;
  std::optional<uint32_t> get_throttle_position_percent() const;

 private:
  const std::string serial_port_path_;
  sp_port *serial_port_ptr_ = 0;
  uint8_t turbine_slave_address_{1};
  bool handle_command_response(const std::string& command,
                               const std::string& response) const;
  void tokenize(const std::string& input_string,
                std::vector<std::string>& tokens,
                char delimiter) const;
  void empty_the_buffer() const;
  std::string read_serial();
};

}  // namespace jet
