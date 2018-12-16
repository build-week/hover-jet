#include "servo_driver.hh"
/*
 * Simple program for testing servos.
 * The following defines should be imported from a config file.
 */

// Frequency that the servos receive updated positions
#define SERVO_FREQ 333.0 // Units: Hz
#define MAX_CONTROLLER_STEP 4095
#define MIN_CONTROLLER_STEP 0
// Pulse sizes
#define MAX_PULSE 2150 // Units: Microseconds
#define MIN_PULSE 875  // Units: Microseconds
// Calculate the minimum and maximum positions
#define SERVO_MAX_STEP MAX_PULSE/((1/SERVO_FREQ * 1000000)/(MAX_CONTROLLER_STEP - MIN_CONTROLLER_STEP))
#define SERVO_MIN_STEP MIN_PULSE/((1/SERVO_FREQ * 1000000)/(MAX_CONTROLLER_STEP - MIN_CONTROLLER_STEP))

ServoDriver driver = ServoDriver("/dev/i2c-1");

int main(void) {
	std::cout << "starting..." << std::endl;
	driver.init();
	driver.setPWMFreq(SERVO_FREQ);
	driver.enableAutoIncrement(true);

	while(1) {
		for (int i = SERVO_MIN_STEP; i < SERVO_MAX_STEP; i+=10)
		{
			driver.setPWM(0, 0, i);
			usleep(1000);
		}
		for (int i = SERVO_MAX_STEP; i > SERVO_MIN_STEP; i-=10)
		{
			driver.setPWM(0, 0, i);
			usleep(1000);
		}
	}
	return 0;
}
