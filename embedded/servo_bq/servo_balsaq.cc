//%bin(servo_balsaq_main)
#include "embedded/servo_bq/servo_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "embedded/servo_bq/servo_pwm_setting_message.hh"

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
  subscriber_ = make_subscriber("servo_command_channel");
  publisher_ = make_publisher("servo_pwm_commands_channel");
  for (auto& servo_config : config) {
    ServoDriver servo = ServoDriver(servo_config.second);
    servos_.push_back(servo);
  }
}

void ServoBq::loop() {
  SetServoMessage rx_message;
  ServoPwmSettingMessage tx_message;

  // subscribe_latest proxy
  bool got_msg = false;
  while (subscriber_->read(rx_message, 1)) {
    got_msg = true;
  }

  if (got_msg) {
    gonogo().go();
    last_msg_recvd_timestamp_ = get_current_time();
    for (uint i = 0; i < rx_message.servo_indices.size(); i++) {
      uint servo_index = rx_message.servo_indices.at(i);
      auto target_radian = rx_message.target_radians.at(i);
      assert(servo_index < servos_.size());
      servos_.at(servo_index).set_vane_angle_radians(target_radian);

      // Record the servo set points for posterity
      tx_message.servo_indices.push_back(servo_index);
      tx_message.pwm_set_points.push_back(servos_.at(servo_index).get_pwm_count());
    }
  }
  publisher_->publish(tx_message);
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last servo command");
  }
}
void ServoBq::shutdown() {
  std::cout << "Servo process shutting down." << std::endl;
  servos_.at(0).shutdown_pwm();
}
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
