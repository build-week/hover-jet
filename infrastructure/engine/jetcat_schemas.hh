#pragma once

#include "engine_enums.hh"

#include <stdint.h>
#include <string>

namespace jet::JetCat {

struct LiveValues {
  uint32_t turbine_rpm;
  uint32_t exhaust_gas_temperature_c;
  float pump_voltage;
  State turbine_state;
  uint32_t throttle_position_percent;
};

struct SystemStatus {
  OffCondition off_condition;
  uint32_t actual_flight_speed;
  uint32_t proportional_part_of_speed_regulator;
  uint32_t AD_value_of_AirSpeed_input;
  uint32_t AD_Zero_value_of_AirSpeed_input;
};

struct TurbineInfo {
  std::string firmware_version_type;
  std::string version_number;
  uint32_t last_run_time;
  uint32_t total_operation_time;
  uint16_t serial_number;
  std::string turbine_type;
};

struct FuelInfo {
  uint32_t actual_fuel_flow;
  uint32_t rest_volume_in_tank;
  uint32_t set_rpm;
  float actual_battery_voltage;
  uint32_t last_run_time_s;
};

}  // namespace jet::JetCat
