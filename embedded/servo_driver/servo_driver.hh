#pragma once
/**
 * Driver for the DS75K servo
 * (http://www.mksservosusa.com/product.php?productid=187)
 */

//%deps(i2c)

#include <stdint.h>  // uint8_t etc
#include <memory>
#include <string>

#include "embedded/pwm_driver/pwm_driver.hh"
#include "third_party/i2c/i2c.h"

class ServoDriver {
 public:
  ServoDriver(const std::string &config_path);
  void set_percentage(float percentage);
  float get_percentage() const;
  int get_servo_index() const;
  void set_angle_radians(float angle);
  void shutdown_pwm();

 private:
  int servo_index_ = -1;
  std::shared_ptr<PwmDriver> pwm_driver_;
  std::string config_path_;
  // The percentage corresponding to max angle.
  float calibrated_max_;
  // The percentage corresponding to zero angle.
  float calibrated_center_;
  float max_angle_;
  float percentage_;
};
