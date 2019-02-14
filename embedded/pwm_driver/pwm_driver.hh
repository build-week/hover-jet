#pragma once
//%deps(i2c)

#include "third_party/i2c/i2c.h"
#include <stdint.h> // uint8_t etc
#include <string>   // uint8_t etc

class PwmDriver {
public:
  PwmDriver(int i2cHandle);
  void reset(void);
  void set_pwm_freq(float freq);
  void set_pwm(uint8_t pwm_num, uint16_t start, uint16_t stop);
  void set_pin(uint8_t pwm_num, uint16_t val);
  void enable_auto_increment(bool enable);
  void sleep(bool sleep);

private:
  I2CDevice device;
  void set_register_bit(uint8_t reg, uint8_t idx);
  void clear_register_bit(uint8_t reg, uint8_t idx);
  void write_to_register(uint8_t reg, uint8_t value);
};
