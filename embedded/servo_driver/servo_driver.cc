//%deps(yaml-cpp)

#include "embedded/servo_driver/servo_driver.hh" // driver library
#include <unistd.h> // usleep
#include <algorithm> // min
#include <cmath> // floor
#include <string.h> // memset
#include <sstream>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace {
  // 850 to 2150 microseconds for the PWM width, specified by the servo
  constexpr int MIN_COUNTS = 1160;
  constexpr int MAX_COUNTS = 2935;
}

ServoDriver::ServoDriver(const int channel, const std::shared_ptr<PwmDriver> &pwm_driver, const std::string &config_path) : channel_(channel), pwm_driver_(pwm_driver), config_path_(config_path) {
  init();
}

int ServoDriver::init() {
  try {
    YAML::Node config = YAML::LoadFile(config_path_);
    max_angle_ = config["max_angle"].as<int>();
    calibrated_center_ = config["calibrated_center"].as<int>();
    calibrated_max_ = config["calibrated_max"].as<int>();
  } catch (YAML::BadFile e) {
    std::string err = std::string("Could not find YAML file ") + config_path_;
    throw std::runtime_error(err);
  }
  percentage_ = calibrated_center_;
  set_percentage(percentage_);
  return 0;
}

void ServoDriver::set_percentage(int percentage) {
  if (percentage < 0 || percentage > 100) {
    throw std::runtime_error(std::string("Percentage out of range: ") + std::to_string(percentage));
  }
  percentage_ = percentage;
  const int counts_range = MAX_COUNTS - MIN_COUNTS;
  int counts = counts_range * static_cast<float>(percentage) / 100 + MIN_COUNTS;
  pwm_driver_->set_pwm(channel_, 0, counts);
}

int ServoDriver::get_percentage() const {
  return percentage_;
}

void ServoDriver::set_angle(float angle) {
  float angleFraction = static_cast<float>(angle) / max_angle_;
  int half_range = (calibrated_max_ - calibrated_center_);
  set_percentage(calibrated_center_ + half_range * angleFraction);
}

