
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/joystick/joystick.hh"
#include "infrastructure/joystick/joystick_command_message.hh"

#include <unistd.h>
#include <memory>

const uint8_t YAW_AXIS_ID = 2;
const uint8_t PEDAL_BUTTON_COUNT = 0;

std::vector<std::string> devices{"/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "end"};

int main(int argc, char* argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto yaw_publisher = comms_factory->make_publisher("joystick_yaw");

  std::unique_ptr<jet::Joystick> pedals;

  for (auto& device : devices) {
    if (device == "end") {
      pedals = nullptr;
      break;
    }
    pedals = std::make_unique<jet::Joystick>(device);
    if (pedals->get_button_count() == PEDAL_BUTTON_COUNT) {
      break;
    }
  }

  while (true) {
    auto pedal_event = pedals->read_event();
    if (pedal_event) {
      if (pedal_event->event_type == jet::EventType::AXIS) {
        if (pedal_event->axis_id == YAW_AXIS_ID) {
          double yaw = (static_cast<double>(*pedal_event->axis_value)) / (std::numeric_limits<int16_t>::max());
          jet::JoystickCommandMessage joystick_command_message;
          joystick_command_message.command = yaw;
          yaw_publisher->publish(joystick_command_message);
          std::cout << "Yaw set to " << yaw << std::endl;
        }
      }
    }
  }
}
