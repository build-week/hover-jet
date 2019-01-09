#pragma once

#include <stdint.h>
#include <map>
#include <string>

namespace jet::JetCat {

enum class State {
  OFF,
  WAIT_FOR_RPM,
  IGNITE,
  ACCELERATE,
  STABILISE,
  LEARN_HI,
  LEARN_LO,
  UNUSED0,
  SLOW_DOWN,
  UNUSED1,
  AUTOOFF,
  RUN,
  ACCLELERATION_DELAY,
  SPEEDREG,
  TWO_SHAFT_REGULATE,
  PREHEAT1,
  PREHEAT2,
  MAINFSTRT,
  UNUSED2,
  KEROS_FULL_ON
};

enum class OffCondition {
  DESCRIPTION,
  NO_OFF_CONDITION_DEFINED,
  SHUT_DOWN_VIA_RC,
  OVERTEMPERATURE,
  IGNITION_TIMEOUT,
  ACCELERATION_TIME_OUT,
  ACCELERATION_TOO_SLOW,
  OVER_RPM,
  LOW_RPM_OFF,
  LOW_BATTERY,
  AUTO_OFF,
  LOW_TEMPERATURE_OFF,
  HI_TEMP_OFF,
  GLOW_PLUG_DEFECTIVE,
  WATCH_DOG_TIMER,
  FAIL_SAFE_OFF,
  MANUAL_OFF,  // Via GSU
  POWER_FAIL,
  TEMP_SENSOR_FAIL,  // Only during startup
  FUEL_FAIL,
  PROP_FAIL,  // Only on two-shaft engines
  ENGINE_FAIL_2ND_ENGINE,
  DIFFERENTIAL_TO_HIGH_2ND_ENGINE,
  NO_COMMUNICATION_2ND_ENGINE
};

enum class ErrorCode : uint16_t {
  OK = 0,
  UNKNOWN_COMMAND,
  INVALID_PARAMETER_COUNT,
  COMMAND_NOT_ALLOWED_IN_STATE,
  PARAMETER_OUT_OF_RANGE,
  PARAMETER_TOO_LONG,
  UNKNOWN_DATA_FORMAT
};

inline std::string enum_to_string(ErrorCode error_code) {
  static const char* error_code_string[] = {"Success",
                                            "Unknown Command",
                                            "Invalid Parameter Count",
                                            "Command Not Allowed in Current State",
                                            "Parameter Out of Range",
                                            "Parameter Too Long",
                                            "Unknown Data Format"};
  return error_code_string[static_cast<typename std::underlying_type<ErrorCode>::type>(
      error_code)];
}

inline ErrorCode string_to_enum(const std::string& error_string) {
  static std::map<const char*, ErrorCode> error_codes = {
      {"OK", ErrorCode::OK},
      {"UC", ErrorCode::UNKNOWN_COMMAND},
      {"PA", ErrorCode::INVALID_PARAMETER_COUNT},
      {"NA", ErrorCode::COMMAND_NOT_ALLOWED_IN_STATE},
      {"PR", ErrorCode::PARAMETER_OUT_OF_RANGE},
      {"PL", ErrorCode::PARAMETER_TOO_LONG},
      {"DF", ErrorCode::UNKNOWN_DATA_FORMAT}};
  return error_codes[error_string.c_str()];
}

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
  uint32_t actual_battery_voltage;
  uint32_t last_run_time_s;
};
}  // namespace jet::JetCat
