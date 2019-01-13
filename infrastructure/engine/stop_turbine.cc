
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/engine/turbine_ignition_message.hh"

#include <memory>
#include <unistd.h>

int main(int argc, char *argv[]) {
  auto comms_factory = std::make_unique<jet::MqttCommsFactory>();
  auto publisher = comms_factory->make_publisher("/turbine/ignition");
  jet::TurbineIgnitionCommandMessage ignition_command_message;
  ignition_command_message.command = jet::IgnitionCommand::STOP;


  std::cout << "Sending engine stop command." << std::endl;
  usleep(1000000);
  publisher->publish(ignition_command_message);
  std::cout << "Sent engine stop command!" << std::endl;
}
