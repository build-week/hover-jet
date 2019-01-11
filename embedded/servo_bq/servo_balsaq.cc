//%bin(servo_balsaq_main)
#include "embedded/servo_bq/servo_balsaq.hh"
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

namespace {
constexpr int PWM_FREQUENCY = 330;
}

void ServoBq::init(int argc, char *argv[]) {
  assert(argc == 3);
  auto config_path = argv[1];
  int servo_index = atoi(argv[2]);

  int i2cHandle = i2c_open(config_path);
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
  }
  std::unique_ptr<PwmDriver> pwm_driver = std::make_unique<PwmDriver>(i2cHandle);
  pwm_driver->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver->enable_auto_increment(true);
  servo = std::make_unique<ServoDriver>(
      0, std::move(pwm_driver),
      config_path);  // TODO create standard config dir
  const std::string channel_name = "servo_channel_" + std::to_string(servo_index);
  std::cout << "channel name " << channel_name << std::endl;
  subscriber_ = make_subscriber(channel_name);
}

void ServoBq::loop() {
  SetServoMessage message;
  if (subscriber_->read(message, 1)) {
    std::cout << "target angle is " << message.target_angle << std::endl;
    servo->set_angle(message.target_angle);
  }
}
void ServoBq::shutdown() {
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
