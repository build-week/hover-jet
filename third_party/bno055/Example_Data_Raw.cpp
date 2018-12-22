#include <iostream>
#include <unistd.h>

#include "third_party/bno055/RPi_Sensor.h"
#include "third_party/bno055/RPi_BNO055.h"
#include "third_party/bno055/utility/imumaths.h"
#include "third_party/i2c/i2c.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055("/dev/i2c-1");


/**************************************************************************/
/*
        Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
int main(void) {
    /* Initialise the sensor */
    if(!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        std::cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!" << std::endl;
        while(1);
    }

    usleep(1000);

    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    std::cout << "Current Temperature: "<< (int)temp << " C" << std::endl;


    bno.setExtCrystalUse(true);

	while (1) {
	    // Possible vector values can be:
	    // - VECTOR_ACCELEROMETER - m/s^2
	    // - VECTOR_MAGNETOMETER    - uT
	    // - VECTOR_GYROSCOPE         - rad/s
	    // - VECTOR_EULER                 - degrees
	    // - VECTOR_LINEARACCEL     - m/s^2
	    // - VECTOR_GRAVITY             - m/s^2
	    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

	    /* Display the floating point data */
	    std::cout << "X: " << euler.x() <<    " Y: " << euler.y() << " Z: "
		<< euler.z() << "\t\t\n\r";

        usleep(1000*BNO055_SAMPLERATE_DELAY_MS);
	}

}
