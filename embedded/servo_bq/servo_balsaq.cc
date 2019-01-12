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
  assert(argc == 1);
  servo_index = atoi(argv[1]);  // get the servo number in 0...3 from the command line

  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
  }
  std::unique_ptr<PwmDriver> pwm_driver = std::make_unique<PwmDriver>(i2cHandle);
  pwm_driver->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver->enable_auto_increment(true);
  servo = std::make_unique<ServoDriver>(
      0, std::move(pwm_driver),
      "cfg/servo_cfg0.yaml");  // TODO create standard config dir
  subscriber_ = make_subscriber("servo_channel_" + std::to_string(servo_index));
}

void ServoBq::loop() {
  SetServoMessage message;
  if (subscriber_->read(message, 1)) {
    servo->set_angle(message.target_angle);
  }
}
void ServoBq::shutdown() {
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::ServoBq)
