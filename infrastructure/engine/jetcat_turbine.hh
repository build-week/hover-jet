#pragma once

#include "infrastructure/engine/engine_enums.hh"

#include <serial/serial.h>

#include <memory>

namespace jet {

class JetCatTurbine {
 public:
  JetCatTurbine();
  void start_engine() const;
  void stop_engine() const;
  void set_serial_control_mode(bool on) const;
  void set_turbine_rpm(uint32_t target_rpm) const;

  JetCat::SystemStatus get_system_status() const;
  JetCat::LiveValues get_live_values() const;
  JetCat::TurbineInfo get_turbine_info() const;
  JetCat::FuelInfo get_fuel_info() const;

  uint32_t get_RPM() const;
  uint32_t get_EGT() const;
  float get_pump_voltage() const;
  JetCat::State get_turbine_state() const;
  uint32_t get_throttle_position_percent() const;

 private:
  std::unique_ptr<serial::Serial> serial_port_;
  uint8_t turbine_slave_address_{1};
};

}  // namespace jet
