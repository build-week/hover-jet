#pragma once

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

inline std::string enum_to_string(State state) {
  static const char* state_string[] = {  "Off",
                                          "Wait for RPM",
                                          "Ignite",
                                          "Accelerate",
                                          "Stabilize",
                                          "Learn High",
                                          "Learn Low",
                                          "UNUSED0",
                                          "Slow Down",
                                          "UNUSED1",
                                          "Auto Off",
                                          "Run",
                                          "Acceleration Delay",
                                          "Speed Reg",
                                          "Two Shaft Regulate",
                                          "Preheat 1",
                                          "Preheat 2",
                                          "MAINFSTRT",
                                          "UNUSED2",
                                          "Kerosene Full On"};
  return state_string[static_cast<typename std::underlying_type<OffCondition>::type>(
      state)];
}

enum class OffCondition {
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

inline std::string enum_to_string(OffCondition off_condition) {
  static const char* off_condition_string[] = {"No off condition defined",
                                            "Shut down via RC",
                                            "Overtemperature",
                                            "Ignition timeout",
                                            "Acceleration Timeout",
                                            "Acceleration too slow",
                                            "Over RPM",
                                            "Low RPM",
                                            "Low battery",
                                            "Auto off",
                                            "Low temperature",
                                            "High temperature",
                                            "Glow plug defective",
                                            "Watchdog timer",
                                            "Failsafe",
                                            "Manual off",
                                            "Power failure",
                                            "Temperature sensor failure",
                                            "Fuel failure",
                                            "Prop failure",
                                            "Engine failure, 2nd engine",
                                            "Differential too high, 2nd engine"};
  return off_condition_string[static_cast<typename std::underlying_type<OffCondition>::type>(
      off_condition)];
}

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

}  // namespace jet::JetCat
