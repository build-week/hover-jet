//%deps(yaml-cpp)

#include "embedded/servo_driver/servo_driver.hh"  // driver library
#include <string.h>                               // memset
#include <unistd.h>                               // usleep
#include <yaml-cpp/yaml.h>
#include <algorithm>  // max, min
#include <cmath>      // M_PI
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
// 850 to 2150 microseconds for the PWM width, specified by the servo
constexpr int MIN_COUNTS = 1160;
constexpr int MAX_COUNTS = 2935;
constexpr int PWM_FREQUENCY = 330;
float rad_to_deg(float radians) {
  return radians * (180.0 / M_PI);
}
}  // namespace

ServoDriver::ServoDriver(const std::string &config_path) : config_path_(config_path) {
  int i2cHandle = i2c_open("/dev/i2c-1");  // TODO make configurable
  if (i2cHandle == -1) {
    std::string err = std::string("Failed to open i2c") + config_path_;
    throw std::runtime_error(err);
  }

  pwm_driver_ = std::make_shared<PwmDriver>(i2cHandle);
  pwm_driver_->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver_->enable_auto_increment(true);

  try {
    YAML::Node config = YAML::LoadFile(config_path_);
    max_angle_ = config["max_angle"].as<float>();
    calibrated_center_ = config["calibrated_center"].as<float>();
    calibrated_max_ = config["calibrated_max"].as<float>();
    servo_index_ = config["index"].as<int>();
    assert(servo_index_ >= 0);
  } catch (YAML::BadFile e) {
    std::string err = std::string("Could not find YAML file ") + config_path_;
    throw std::runtime_error(err);
  }

  percentage_ = calibrated_center_;
  set_percentage(percentage_);
}

void ServoDriver::set_percentage(float unchecked_percentage) {
  if (unchecked_percentage > 100 || unchecked_percentage < 0) {
    std::cout << "Desired servo percentage out of range: " << unchecked_percentage
              << std::endl;
  }
  percentage_ = std::max(std::min(100.0f, unchecked_percentage), 0.0f);

  const float counts_range = MAX_COUNTS - MIN_COUNTS;
  int counts = round(counts_range * percentage_ / 100 + MIN_COUNTS);
  pwm_driver_->set_pwm(servo_index_, 0, counts);
}

float ServoDriver::get_percentage() const {
  return percentage_;
}

int ServoDriver::get_servo_index() const {
  return servo_index_;
}

void ServoDriver::set_angle_radians(
    float angle_radians) {  // TODO noble to give units to angle
  float angle_degrees = rad_to_deg(angle_radians);
  float angleFraction = static_cast<float>(angle_degrees) / max_angle_;
  float half_range = (calibrated_max_ - calibrated_center_);
  set_percentage(calibrated_center_ + half_range * angleFraction);
}

void ServoDriver::shutdown_pwm() {
  pwm_driver_->sleep(true);
}
