
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/engine/throttle_command_message.hh"
#include "infrastructure/engine/turbine_ignition_message.hh"
#include "infrastructure/joystick/joystick.hh"

#include <unistd.h>
#include <memory>

const uint8_t ENGINE_STOP_BUTTON_ID = 17;
const uint8_t ENGINE_START_BUTTON_ID = 30;
const uint8_t THROTTLE_AXIS_ID = 2;
const uint8_t THROTTLE_BUTTON_COUNT = 32;

std::vector<std::string> devices{"/dev/input/js0", "/dev/input/js1", "/dev/input/js2", "none"};

int main(int argc, char* argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto throttle_publisher = comms_factory->make_publisher("turbine_set_throttle");
  auto ignition_publisher = comms_factory->make_publisher("turbine_ignition");

  std::unique_ptr<jet::Joystick> throttle;

  for (auto& device : devices) {
    if (device == "end") {
      throttle = nullptr;
      break;
    }
    throttle = std::make_unique<jet::Joystick>(device);
    if (throttle->get_button_count() == THROTTLE_BUTTON_COUNT) {
      break;
    }
  }

  while (true) {
    auto throttle_event = throttle->read_event();
    if (throttle_event) {
      switch (throttle_event->event_type) {
        case jet::EventType::BUTTON: {
          if (throttle_event->axis_id == ENGINE_START_BUTTON_ID) {
            if (*throttle_event->button_state == jet::ButtonState::RELEASED) {
              jet::TurbineIgnitionCommandMessage ignition_command_message;
              ignition_command_message.command = jet::IgnitionCommand::START;
              ignition_publisher->publish(ignition_command_message);
              std::cout << "Engine Start" << std::endl;
            }
          } else if (throttle_event->axis_id == ENGINE_STOP_BUTTON_ID) {
            if (*throttle_event->button_state == jet::ButtonState::RELEASED) {
              jet::TurbineIgnitionCommandMessage ignition_command_message;
              ignition_command_message.command = jet::IgnitionCommand::STOP;
              ignition_publisher->publish(ignition_command_message);
              std::cout << "Engine Stop" << std::endl;
            }
          } else {
            std::cerr << "Unrecognized button input on button " << unsigned(throttle_event->axis_id) << std::endl;
          }
          break;
        }
        case jet::EventType::AXIS: {
          if (throttle_event->axis_id == THROTTLE_AXIS_ID) {
            uint32_t throttle_percent = 100 - 100 * ((*throttle_event->axis_value) + INT16_MAX) / (INT16_MAX * 2);
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
}
