//%deps(yaml-cpp)

#include "embedded/servo_driver/servo_driver.hh"  // driver library
#include <string.h>                               // memset
#include <unistd.h>                               // usleep
#include <yaml-cpp/yaml.h>
#include <algorithm>  // min
#include <cmath>      // floor
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
// 850 to 2150 microseconds for the PWM width, specified by the servo
constexpr int MIN_COUNTS = 1160;
constexpr int MAX_COUNTS = 2935;
constexpr int PWM_FREQUENCY = 330;
}  // namespace

ServoDriver::ServoDriver(const std::string &config_path) : config_path_(config_path) {
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::string err = std::string("Failed to open i2c") + config_path_;
    throw std::runtime_error(err);
  }

  pwm_driver_ = std::make_shared<PwmDriver>(i2cHandle);
  pwm_driver_->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver_->enable_auto_increment(true);

  try {
    YAML::Node config = YAML::LoadFile(config_path_);
    max_angle_ = config["max_angle"].as<int>();
    calibrated_center_ = config["calibrated_center"].as<int>();
    calibrated_max_ = config["calibrated_max"].as<int>();
    servo_index_ = config["index"].as<int>();
  } catch (YAML::BadFile e) {
    std::string err = std::string("Could not find YAML file ") + config_path_;
    throw std::runtime_error(err);
  }
  percentage_ = calibrated_center_;
  set_percentage(percentage_);
}



void ServoDriver::set_percentage(int percentage) {
  if (percentage < 0 || percentage > 100) {
    throw std::runtime_error(std::string("Percentage out of range: ") +
                             std::to_string(percentage));
  }
  percentage_ = percentage;
  const int counts_range = MAX_COUNTS - MIN_COUNTS;
  int counts = counts_range * static_cast<float>(percentage) / 100 + MIN_COUNTS;
  pwm_driver_->set_pwm(servo_index_, 0, counts);
}

int ServoDriver::get_percentage() const {
  return percentage_;
}

int ServoDriver::get_servo_index() const {
  return servo_index_;
}

void ServoDriver::set_angle(float angle) {  // TODO noble to give units to angle
  float angleFraction = static_cast<float>(angle) / max_angle_;
  int half_range = (calibrated_max_ - calibrated_center_);
  set_percentage(calibrated_center_ + half_range * angleFraction);
}
