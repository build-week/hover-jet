
#include "infrastructure/joystick/joystick.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/engine/throttle_command_message.hh"
#include "infrastructure/engine/turbine_ignition_message.hh"

#include <unistd.h>
#include <memory>

const uint8_t ENGINE_STOP_BUTTON_ID = 17;
const uint8_t ENGINE_START_BUTTON_ID = 30;
const uint8_t THROTTLE_AXIS_ID = 2;

int main(int argc, char *argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto throttle_publisher = comms_factory->make_publisher("turbine_set_throttle");
  auto ignition_publisher = comms_factory->make_publisher("turbine_ignition");

  jet::Joystick joystick("/dev/input/js0");
  while (true) {
    auto event = joystick.read_event();
    if (!event) {
      continue;
    }
    switch (event->event_type) {
      case jet::EventType::BUTTON: {
        if (event->axis_id == ENGINE_START_BUTTON_ID) {
          if (*event->button_state == jet::ButtonState::RELEASED) {
            jet::TurbineIgnitionCommandMessage ignition_command_message;
            ignition_command_message.command = jet::IgnitionCommand::START;
            ignition_publisher->publish(ignition_command_message);
            std::cout << "Engine Start" << std::endl;
          }
        }
        else if (event->axis_id == ENGINE_STOP_BUTTON_ID) {
          if ( *event->button_state == jet::ButtonState::RELEASED) {
            jet::TurbineIgnitionCommandMessage ignition_command_message;
            ignition_command_message.command = jet::IgnitionCommand::STOP;
            ignition_publisher->publish(ignition_command_message);
            std::cout << "Engine Stop" << std::endl;
          }
        }
        else {
          std::cerr << "Unrecognized button input on button " << unsigned(event->axis_id) << std::endl;
        }
        break;
      }
      case jet::EventType::AXIS: {
        if (event->axis_id == THROTTLE_AXIS_ID)
        {
          uint32_t throttle_percent = 100 - 100 * ((*event->axis_value) + INT16_MAX) / (INT16_MAX * 2);
          jet::ThrottleCommandMessage throttle_command_message;
          throttle_command_message.throttle_percent = throttle_percent;
          throttle_publisher->publish(throttle_command_message);
          std::cout << "Throttle set to " << throttle_percent << "%" << std::endl;
        }
        break;
      }
      default:
        break;
    }
  }
}
