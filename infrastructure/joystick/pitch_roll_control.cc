
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/joystick/joystick.hh"
#include "infrastructure/joystick/joystick_command_message.hh"

#include <unistd.h>
#include <memory>

const uint8_t PITCH_AXIS_ID = 1;
const uint8_t ROLL_AXIS_ID = 0;
const uint8_t JOYSTICK_BUTTON_COUNT = 19;

std::vector<std::string> devices{"/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "end"};

int main(int argc, char* argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto pitch_publisher = comms_factory->make_publisher("/joystick/pitch");
  auto roll_publisher = comms_factory->make_publisher("/joystick/roll");

  std::unique_ptr<jet::Joystick> joystick;

  for (auto& device : devices) {
    if (device == "end") {
      joystick = nullptr;
      break;
    }
    joystick = std::make_unique<jet::Joystick>(device);
    if (joystick->get_button_count() == JOYSTICK_BUTTON_COUNT) {
      break;
    }
  }

  while (true) {
    auto joystick_event = joystick->read_event();
    if (joystick_event) {
      if (joystick_event->event_type == jet::EventType::AXIS) {
        if (joystick_event->axis_id == PITCH_AXIS_ID) {
          double pitch = -1 * ((double)(*joystick_event->axis_value)) / (INT16_MAX);
          jet::JoystickCommandMessage joystick_command_message;
          joystick_command_message.command = pitch;
          pitch_publisher->publish(joystick_command_message);
          std::cout << "Pitch set to " << pitch << std::endl;
        }
        if (joystick_event->axis_id == ROLL_AXIS_ID) {
          double roll = ((double)(*joystick_event->axis_value)) / (INT16_MAX);
          jet::JoystickCommandMessage joystick_command_message;
          joystick_command_message.command = roll;
          roll_publisher->publish(joystick_command_message);
          std::cout << "Roll set to " << roll << std::endl;
        }
      }
    }
  }
}
