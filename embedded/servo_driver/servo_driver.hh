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
  ServoDriver(const int channel,
              const std::shared_ptr<PwmDriver> &driver,
              const std::string &config_path);
  int init();
  void set_percentage(int percentage);
  int get_percentage() const;
  void set_angle(float angle);

 private:
  int channel_;
  const std::shared_ptr<PwmDriver> &pwm_driver_;
  std::string config_path_;
  // The percentage corresponding to max angle.
  int calibrated_max_;
  // The percentage corresponding to zero angle.
  int calibrated_center_;
  int max_angle_;
  int percentage_;
};
