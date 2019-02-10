
#include "infrastructure/joystick/joystick.hh"

#include <fcntl.h>
#include <unistd.h>

namespace jet {

Joystick::Joystick(std::string_view joystick_path) : joystick_path_(joystick_path) {
  joystick_fd_ = open(joystick_path_.data(), O_RDONLY);
  if (joystick_fd_ == -1) {
    std::cerr << "Could not open joystick device." << std::endl;
  }
}

Joystick::~Joystick() {
  close(joystick_fd_);
}

std::optional<JoystickEvent> Joystick::read_event() {
  size_t bytes = read(joystick_fd_, &event_buffer_, sizeof(&event_buffer_));
  if (bytes == sizeof(&event_buffer_)) {
    JoystickEvent event;
    event.timestamp_ms = event_buffer_.time;
    if (!int_to_enum(event_buffer_.type, event.event_type)) {
      return {};
    }

    if (event.event_type == EventType::BUTTON) {
      event.axis_id = event_buffer_.number;
      ButtonState button_state;
      int_to_enum(event_buffer_.value, button_state);
      event.button_state = button_state;
      return event;
    }

    if (event.event_type == EventType::AXIS) {
      event.axis_id = event_buffer_.number;
      event.axis_value = event_buffer_.value;
      return event;
    }
  }
  return {};
}

uint8_t Joystick::get_axis_count() {
  uint8_t axis_count;
  if (ioctl(joystick_fd_, JSIOCGAXES, &axis_count) == -1) {
    return 0;
  }
  return axis_count;
}

uint8_t Joystick::get_button_count() {
  uint8_t button_count;
  if (ioctl(joystick_fd_, JSIOCGBUTTONS, &button_count) == -1) {
    return 0;
  }
  return button_count;
}

}  // namespace jet
