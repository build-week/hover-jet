//%bin(servo_balsaq_main)
#include "embedded/servo_bq/servo_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

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

void ServoBq::init(const Config& config) {
  subscriber = make_subscriber("servo_command_channel");
  for (auto& servo_config : config) {
    ServoDriver servo = ServoDriver(servo_config.second);
    servos.push_back(servo);
  }
}

void ServoBq::loop() {
  SetServoMessage message;

  // subscribe_latest proxy
  bool got_msg = false;
  while (subscriber->read(message, 1)) {
    got_msg = true;
  }

  if (got_msg) {
    gonogo().go();
    last_msg_recvd_timestamp_ = get_current_time();
    for (uint i = 0; i < message.servo_indices.size(); i++) {
      auto servo_index = message.servo_indices.at(i);
      auto target_radian = message.target_radians.at(i);
      assert(servo_index < servos.size());
      servos.at(servo_index).set_angle_radians(target_radian);
    }
  }
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last servo command");
  }
}
void ServoBq::shutdown() {
  std::cout << "Servo process shutting down." << std::endl;
  servos.at(0).shutdown_pwm();
}
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
