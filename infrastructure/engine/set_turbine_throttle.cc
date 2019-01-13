
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/engine/throttle_command_message.hh"

#include <memory>
#include <unistd.h>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Set Turbine Throttle tool takes one argument. An integer between 0 and 100." << std::endl;
    exit(1);
  }

  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto publisher = comms_factory->make_publisher("/turbine/set_throttle");
  jet::ThrottleCommandMessage throttle_command_message;
  throttle_command_message.throttle_percent = std::atoi(argv[1]);

  std::cout << "Setting throttle to " << throttle_command_message.throttle_percent << "%." << std::endl;
  usleep(1000000);
  publisher->publish(throttle_command_message);
  std::cout << "Sent engine stop command!" << std::endl;
}
