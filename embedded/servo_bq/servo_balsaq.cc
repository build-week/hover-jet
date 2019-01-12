//%bin(servo_balsaq_main)
#include "embedded/servo_bq/servo_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
// #include "servo_driver.hh"

#include <cassert>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

void ServoBq::init(int argc, char *argv[]) {
  assert(argc == 2);
  std::string config_path = argv[1];
  servo = std::make_unique<ServoDriver>(config_path);  // TODO create standard config dir

  const std::string channel_name = "servo_channel_" + std::to_string(servo->get_servo_index());
  std::cout << "channel name " << channel_name << std::endl;
  subscriber_ = make_subscriber(channel_name);

}

void ServoBq::loop() {
  SetServoMessage message;
  if (subscriber_->read(message, 1)) {
    std::cout << "target is " << message.target_angle << std::endl;
    current_target_percentage = message.target_angle;
    std::cout << message.target_angle << std::endl;
    servo->set_percentage(current_target_percentage);
  }
  // std::cout << "about to command servo" << std::endl;
  // std::cout << " commanded" << std::endl;
}
void ServoBq::shutdown() {
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
