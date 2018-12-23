#pragma once
/**
 * Driver for the DS75K servo (http://www.mksservosusa.com/product.php?productid=187)
*/

//%deps(i2c)

#include <stdint.h> // uint8_t etc

#include "embedded/pwm_driver/pwm_driver.hh"
#include "third_party/i2c/i2c.h"


class ServoDriver {
public:
	ServoDriver(char* dev);
	int init();
	void set_percentage(int channel, int percentage);

private:
        PwmDriver pwm_driver_;
};
