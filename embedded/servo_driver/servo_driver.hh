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
#include "infrastructure/config/config.hh"
#include "third_party/i2c/i2c.h"

class ServoDriver {
 public:
  ServoDriver(const Config& config);
  void set_percentage(double percentage, uint max_pwm_count = MAX_PWM_COUNTS, uint min_pwm_count = MIN_PWM_COUNTS);
  double get_percentage() const;
  uint get_pwm_count() const;
  int get_servo_index() const;
  void set_vane_angle_radians(double angle);
  void shutdown_pwm();

 private:

  // 850 to 2150 microseconds for the PWM width, specified by the servo
  // Values checked on Oscilloscope
  static constexpr uint MIN_PWM_COUNTS = 1160;
  static constexpr uint MAX_PWM_COUNTS = 2935;
  static constexpr uint PWM_FREQUENCY = 330;
  int servo_index_ = -1;
  std::shared_ptr<PwmDriver> pwm_driver_;
  std::string config_path_;
  // The count corresponding to max angle.
  uint calibrated_max_pwm_count_;
  // The count corresponding to zero angle.
  uint calibrated_min_pwm_count_;
  // Max angle in radians that the servo can achieve.
  double max_angle_radians_;
  double percentage_;
  uint pwm_count_;
};
