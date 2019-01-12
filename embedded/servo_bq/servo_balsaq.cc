//%bin(servo_balsaq_main)
#include "embedded/servo_bq/servo_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
// #include "servo_driver.hh"

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

namespace {
std::string get_channel_name_from_servo_index(int servo_index) {
  const std::string channel_name = "servo_channel_" + std::to_string(servo_index);
  return channel_name;
}
}  // namespace

void ServoBq::init(int argc, char* argv[]) {
  assert(argc > 1);
  const int n_servos = argc - 1;
  // ServoDriver servos [n_servos];
  for (int i = 0; i < n_servos; i++) {
    std::string config_path = argv[1 + i];
    ServoDriver servo = ServoDriver(config_path);  // TODO create standard config dir
    // servos.push_back(servo);
    SubscriberPtr subscriber = make_subscriber(get_channel_name_from_servo_index(i));
    // subscribers.push_back(std::move(subscriber));
    servo_subscriber_pairs.push_back(std::make_pair(servo, std::move(subscriber)));
  }
}

void ServoBq::loop() {
  SetServoMessage message;
  for (auto const& servo_subscriber_pair : servo_subscriber_pairs) {
    auto servo = servo_subscriber_pair.first;
    auto &subscriber = servo_subscriber_pair.second;
    if (subscriber->read(message, 1)) {
      std::cout << "target is " << message.target_angle << std::endl;
      current_target_percentage = message.target_angle;
      std::cout << message.target_angle << std::endl;
      servo.set_percentage(current_target_percentage);
    }
  }
}
// std::cout << "about to command servo" << std::endl;
// std::cout << " commanded" << std::endl;
void ServoBq::shutdown() {
}
}  // namespace jet


BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
