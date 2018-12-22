#pragma once

#include <stdint.h> // uint8_t etc
#include "embedded/pwm_driver/pwm_driver.hh"
#include "third-party/i2c/i2c.h"

class ServoDriver {
public:
	ServoDriver(char* dev);
	int init();
	void set_percentage(int channel, int percentage);

private:
        PwmDriver pwm_driver_;
};
