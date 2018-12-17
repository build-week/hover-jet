#include "servo_driver.hh"

int main(void) {
	std::cout << "starting..." << std::endl;
	ServoDriver driver = ServoDriver("/dev/i2c-1");
	driver.init();
	driver.set_pwm_freq(50.0);
	driver.enable_auto_increment(true);
	// driver.setPWM(0,0,100);

	while(1) {
		for (int i=0; i<4096; i++) {
			driver.set_pwm(0, 0, i);
			usleep(1000);
		}
	}
	return 0;
}