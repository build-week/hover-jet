#include "embedded/servo_driver/servo_driver.hh" // driver library
#include <unistd.h> // usleep
#include <algorithm> // min
#include <cmath> // floor
#include <string.h> // memset

ServoDriver::ServoDriver(char* dev) : pwm_driver_(dev) {}
namespace {
  constexpr int PWM_FREQUENCY = 333;
  // 850 to 2150 microseconds for the PWM width, specified by the servo
  constexpr int MIN_COUNTS = 1160;
  constexpr int MAX_COUNTS = 2935;
}

int ServoDriver::init() {
  pwm_driver_.init();
  pwm_driver_.set_pwm_freq(PWM_FREQUENCY);
  pwm_driver_.enable_auto_increment(true);
  return 0;
}

void ServoDriver::set_percentage(int channel, int percentage) {
  // TODO(dye): Check inputs.
  const int counts_range = MAX_COUNTS - MIN_COUNTS;
  int counts = counts_range * static_cast<float>(percentage) / 100 + MIN_COUNTS;
  pwm_driver_.set_pwm(channel, 0, counts);
}

