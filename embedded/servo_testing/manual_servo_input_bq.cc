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

namespace {
constexpr int PWM_FREQUENCY = 330;
}

void SingleServoCommandBq::init(int argc, char *argv[]) {
  assert(argc == 3);
  int servo_index = atoi(argv[1]);
  float target_radian = (float)(atoi(argv[2]));

  const std::string channel_name = "servo_command_channel";
  std::cout << "channel name " << channel_name << std::endl;
  publisher_ = make_publisher(channel_name);
  SetServoMessage set_servo_message;
  std::vector<float> target_radians;
  target_radians.push_back(target_radian);
  set_servo_message.target_radians = target_radians;

  std::vector<int> servo_indices;
  servo_indices.push_back(servo_index);
  set_servo_message.servo_indices = servo_indices;

  std::cout << "enter to send command" << std::endl;
  std::cin.ignore();
  publisher_->publish(set_servo_message);
  exit(0); // What's the right way to shutdown a bq?
}

void SingleServoCommandBq::loop() {
}
void SingleServoCommandBq::shutdown() {
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::SingleServoCommandBq)
