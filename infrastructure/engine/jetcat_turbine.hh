#pragma once

#include "infrastructure/engine/engine_enums.hh"

#include <serial/serial.h>

#include <memory>
#include <optional>

namespace jet {

class JetCatTurbine {
 public:
  JetCatTurbine();
  bool start_engine() const;
  bool stop_engine() const;
  void set_serial_control_mode(bool on) const;
  bool set_turbine_rpm(uint32_t target_rpm) const;

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
  std::unique_ptr<serial::Serial> serial_port_;
  uint8_t turbine_slave_address_{1};
  bool handle_command_response(const std::string& command,
                               const std::string& response) const;
  void tokenize(const std::string& str,
                std::vector<std::string>& tokens,
                char delim) const;
  void empty_the_buffer() const;
};

}  // namespace jet
