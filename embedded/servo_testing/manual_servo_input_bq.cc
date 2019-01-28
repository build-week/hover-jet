//%bin(manual_servo_command_main)
#include "embedded/servo_testing/manual_servo_input_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

SingleServoCommandBq::SingleServoCommandBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void SingleServoCommandBq::init(int argc, char *argv[]) {
  assert(argc == 3);
  servo_index = atoi(argv[1]);
  target_angle = (float)(atoi(argv[2]));

  const std::string channel_name = "servo_channel_" + std::to_string(servo_index);
  std::cout << "channel name " << channel_name << std::endl;
  publisher_ = make_publisher(channel_name);

}

void SingleServoCommandBq::loop() {
  SetServoMessage set_servo_message;
  set_servo_message.target_angle = target_angle;
  publisher_->publish(set_servo_message);
  shutdown();

}
void SingleServoCommandBq::shutdown() {
  exit(0);
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::SingleServoCommandBq)
