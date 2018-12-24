#pragma once
//%deps(i2c)

#include <stdint.h> // uint8_t etc
#include <string> // uint8_t etc
#include "third_party/i2c/i2c.h"

class PwmDriver {
public:
	PwmDriver(const std::string &dev);
	uint8_t init();
	void reset(void);
	void set_pwm_freq(float freq);
	void set_pwm(uint8_t pwm_num, uint16_t start, uint16_t stop);
	void set_pin(uint8_t pwm_num, uint16_t val);
	void enable_auto_increment(bool enable);
private:
	const char* device_name;
	I2CDevice device;
	void set_register_bit(uint8_t reg, uint8_t idx);
	void clear_register_bit(uint8_t reg, uint8_t idx);
	void write_to_register(uint8_t reg, uint8_t value);
};
