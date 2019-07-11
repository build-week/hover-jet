#include <readline/readline.h>
#include <unistd.h>  // usleep
#include <iostream>
#include <vector>
#include "servo_driver.hh"

constexpr int PWM_FREQUENCY = 330;

int main() {
  Config servo_configs = YAML::LoadFile("cfg/servo_configs.yaml");
  ServoDriver servo1(servo_configs[0]);

  std::vector<ServoDriver> servos;
  servos.push_back(servo1);

  // Sweep through servos and set to -max_angle, 0, max_angle
  for (auto &servo : servos) {
    servo.set_vane_angle_radians(0.174533);
  }
  usleep(1000000);
  for (auto &servo : servos) {
    servo.set_vane_angle_radians(0.0);
  }
  usleep(1000000);
  for (auto &servo : servos) { 
    servo.set_vane_angle_radians(-0.174533);
  }
  return 0;
}
