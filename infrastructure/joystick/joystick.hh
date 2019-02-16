
#pragma once

#include <linux/joystick.h>
#include <cstdint>

#include <iostream>
#include <optional>
#include <string>

namespace jet {

enum class EventType { BUTTON, AXIS, INIT };

inline bool int_to_enum(uint8_t enum_int_value, EventType& event_type_value) {
  if (enum_int_value == 1) {
    event_type_value = EventType::BUTTON;
    return true;
  }
  if (enum_int_value == 2) {
    event_type_value = EventType::AXIS;
    return true;
  }
  if (enum_int_value == 128) {
    event_type_value = EventType::INIT;
    return true;
  }
  return false;
}

enum class ButtonState { PRESSED, RELEASED };

inline bool int_to_enum(int16_t enum_int_value, ButtonState& button_state_value) {
  if (enum_int_value == 0) {
    button_state_value = ButtonState::PRESSED;
    return true;
  }
  if (enum_int_value == 1) {
    button_state_value = ButtonState::RELEASED;
    return true;
  }
  return false;
}

struct JoystickEvent {
  uint32_t timestamp_ms;
  EventType event_type;
  uint8_t axis_id;
  std::optional<ButtonState> button_state;
  std::optional<int16_t> axis_value;
};

class Joystick {
 public:
  Joystick(std::string_view joystick_path);
  ~Joystick();
  std::optional<JoystickEvent> read_event();
  uint8_t get_axis_count();
  uint8_t get_button_count();

 private:
  const std::string serial_port_path_;
  const std::string_view joystick_path_;
  int joystick_fd_;
  struct js_event event_buffer_;
};

}  // namespace jet
