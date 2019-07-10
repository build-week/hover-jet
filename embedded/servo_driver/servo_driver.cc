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

ServoDriver::ServoDriver(const Config& config) {
  int i2cHandle = i2c_open("/dev/i2c-1");  // TODO make configurable
  if (i2cHandle == -1) {
    std::string err = std::string("Failed to open i2c");
    throw std::runtime_error(err);
  }

  pwm_driver_ = std::make_shared<PwmDriver>(i2cHandle);
  pwm_driver_->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver_->enable_auto_increment(true);

  try {
    max_angle_radians_ = config["max_angle_radians"].as<double>();
    calibrated_min_pwm_count_ = config["calibrated_min_pwm_count"].as<uint>();
    calibrated_max_pwm_count_ = config["calibrated_max_pwm_count"].as<uint>();
    servo_index_ = config["index"].as<int>();
    assert(servo_index_ >= 0);
  } catch (YAML::BadFile e) {
    std::string err = std::string("Could not find YAML file ");
    throw std::runtime_error(err);
  }

  set_angle_radians(0);
}

void ServoDriver::set_percentage(double unchecked_percentage, uint max_pwm_count, uint min_pwm_count) {
  if (unchecked_percentage > 1 || unchecked_percentage < 0) {
    std::cout << "Desired servo percentage out of range: " << unchecked_percentage
              << std::endl;
  }
  percentage_ = std::max(std::min(1.0, unchecked_percentage), 0.0);

  const uint counts_range = max_pwm_count - min_pwm_count;
  const uint counts = counts_range * percentage_ + min_pwm_count;
  pwm_count_ = counts;
  pwm_driver_->set_pwm(servo_index_, 0, counts);
}

double ServoDriver::get_percentage() const {
  return percentage_;
}

uint ServoDriver::get_pwm_count() const {
  return pwm_count_;
}

int ServoDriver::get_servo_index() const {
  return servo_index_;
}

void ServoDriver::set_angle_radians(double angle_radians) {
  const double slope = 1.0 / (2*max_angle_radians_);
  const double percentage = slope * (angle_radians + max_angle_radians_);
  set_percentage(percentage, calibrated_max_pwm_count_, calibrated_min_pwm_count_);
}

void ServoDriver::shutdown_pwm() {
  pwm_driver_->disable_servos(true);
}
