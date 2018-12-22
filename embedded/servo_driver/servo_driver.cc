/***
Servo driver library for Jet hover project
@author Jasbir Harnal jaz.jlh@gmail.com

Borrows from:
Adafruit PCA9685 library: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/

***/

#include "embedded/servo_driver/servo_driver.hh" // driver library
#include <unistd.h> // usleep
#include <algorithm> // min
#include <cmath> // floor
#include <string.h> // memset

namespace {
}
ServoDriver::ServoDriver(char* dev) : pwm_driver_(dev) {}

int ServoDriver::init() {
  pwm_driver_.init();
  pwm_driver_.set_pwm_freq(333);
  pwm_driver_.enable_auto_increment(true);
}

void ServoDriver::set_percentage(int channel, int percentage) {
  // TODO(dye): Check inputs.
  constexpr int MIN_COUNT = 1160;
  constexpr int MAX_COUNT = 2935;
  const int count_range = MAX_COUNT - MIN_COUNT;
  int clock_count = count_range * static_cast<float>(percentage) / 100 + MIN_COUNT;
  pwm_driver_.set_pwm(channel, 0, clock_count);
}

