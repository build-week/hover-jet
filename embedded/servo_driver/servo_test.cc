#include "servo_driver.hh"

ServoDriver driver = ServoDriver("/dev/i2c-1");

int main(void) {
	std::cout << "starting..." << std::endl;
	driver.init();
	driver.setPWMFreq(50.0);
	driver.enableAutoIncrement(true);
	// driver.setPWM(0,0,100);

	while(1) {
		for (int i=0; i<4096; i++) {
			driver.setPWM(0, 0, i);
			usleep(1000);
		}
	}
	return 0;
}