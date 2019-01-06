#include <readline/readline.h>
#include <unistd.h>  // usleep
#include <iostream>
#include <vector>
#include "servo_driver.hh"

constexpr int PWM_FREQUENCY = 330;

int main() {
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
    return -1;
  }
  std::shared_ptr<PwmDriver> pwm_driver = std::make_shared<PwmDriver>(i2cHandle);
  pwm_driver->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver->enable_auto_increment(true);

  ServoDriver servo1(0, pwm_driver, "cfg/servo_cfg0.yaml");
  ServoDriver servo2(1, pwm_driver, "cfg/servo_cfg1.yaml");
  ServoDriver servo3(2, pwm_driver, "cfg/servo_cfg2.yaml");
  ServoDriver servo4(3, pwm_driver, "cfg/servo_cfg3.yaml");

  std::vector<ServoDriver> servos;
  servos.push_back(servo1);
  servos.push_back(servo2);
  servos.push_back(servo3);
  servos.push_back(servo4);

  // Sweep through servos and set to -max_angle, 0, max_angle
  for (auto &servo : servos) {
    servo.set_angle(-20.0f);
  }
  usleep(1000000);
  for (auto &servo : servos) {
    servo.set_angle(-0.f);
  }
  usleep(1000000);
  for (auto &servo : servos) {
    servo.set_angle(20.f);
  }
  return 0;
}
