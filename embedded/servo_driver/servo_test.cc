#include "servo_driver.hh"
#include <iostream>
#include <unistd.h> // usleep

int main(void) {
	ServoDriver driver = ServoDriver("/dev/i2c-1");
	driver.init();
        const uint8_t channel = 1;
        std::cout << "sweeping through servo" << std::endl;
        for (int i = 0; i <= 100; ++i) {
          driver.set_percentage(channel, i);
          usleep(10000);
        }
	return 0;
}
