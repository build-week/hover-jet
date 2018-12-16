/***
Servo driver library for Jet hover project
@author Jasbir Harnal jaz.jlh@gmail.com

Borrows from:
Adafruit PCA9685 library: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/
Other help:

***/

#include "servo_driver.hh"

ServoDriver::ServoDriver(char* dev) {
	deviceName = dev;
}

uint8_t ServoDriver::init() {
	int i2cbus = i2c_open(deviceName);
	if (i2cbus == -1) {
		// we could not open the interface
		std::cout << "could not open interface " << deviceName << ", error: " << std::strerror(errno) << std::endl;
		return 1;
	}
	// allocate memory for the device struct
	memset(&device, 0, sizeof(device));

	device.bus = i2cbus;
	device.addr = PCA9685_I2C_ADDR;
	device.iaddr_bytes = 1;			// size of internal addresses = 1 byte
	device.page_bytes = 16; 		// device is capapble of 16 bytes per page

	// first we want to reset the device
	reset();
	// then set a default frequency
	setPWMFreq(100);
	return 0;
}

void ServoDriver::reset() {
	setRegisterBit(PCA9685_MODE1, 7);
	usleep(10000);
}

// @brief Sets the PWM frequency for the chip - max ~1.6kHz
void ServoDriver::setPWMFreq(float freq) {
	std::cout << "setting PWM frequency to " << freq << std::endl;
	freq *= 0.94; // this is necessary according to the Adafruit lib
	// this formula is on page 25 of the PCA9685 datasheet
	float prescaleval = 25000000; // 25Mhz oscillator clock freq
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	uint8_t prescale = floor(prescaleval + 0.5);

	// We need to go into sleep mode before setting the prescaler
	setRegisterBit(PCA9685_MODE1, 4);					//go to sleep
	writeToRegister(PCA9685_PRESCALE, prescale);		//write prescale to register
	clearRegisterBit(PCA9685_MODE1, 4);				    //bring it out of sleep mode
	usleep(5000);
	enableAutoIncrement(true);							//set the mode to auto increment
}

//@brief Sets the PWM output of one of the PCA9685 pins
void ServoDriver::setPWM(uint8_t servo_num, uint16_t start, uint16_t stop) {
	std::cout << "setting PWM from " << start << "-" << stop << std::endl;
	//first we want to clamp the values between 0-4095
	start = std::min(start, (uint16_t)4095);
	stop = std::min(stop, (uint16_t)4095);
	// base address is LED0_ON_L, each servo has a 4 byte register size
	// calculate the address based on the servo number
	uint8_t base_reg = LED0_ON_L + 4*servo_num;

	// fill in the registers
	writeToRegister(base_reg, start);
	writeToRegister(base_reg+1, start >> 8);
	writeToRegister(base_reg+2, stop);
	writeToRegister(base_reg+3, stop >> 8);

	// write(i2cbus, &addr, 1);
	// // send the start and stop values to the appropriate registers
	// write(i2cbus, &start, 1);
	// start = start >> 8;
	// write(i2cbus, &start, 1);
	// write(i2cbus, &stop, 1);
	// stop = stop >> 8;
	// write(i2cbus, &stop, 1);
}

void ServoDriver::enableAutoIncrement(bool enable) {
	if (enable) {
		setRegisterBit(PCA9685_MODE1, 5);
	}
	else {
		clearRegisterBit(PCA9685_MODE1, 5);
	}
}

void ServoDriver::setRegisterBit(uint8_t reg, uint8_t idx) {
	std::cout << "setting reg " << unsigned(reg) << " bit " << unsigned(idx) << " to 1" << std::endl;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, sizeof(buffer));

	if(i2c_read(&device, reg, buffer, size) != size) {
		std::cout << "error reading from " << reg << " register" << std::endl;
	}
	buffer[0] = (1 << idx) | buffer[0];
	if (i2c_write(&device, reg, buffer, size) != size) {
		std::cout << "error writing to " << reg << " register" << std::endl;
	}
}

void ServoDriver::clearRegisterBit(uint8_t reg, uint8_t idx) {
	std::cout << "setting reg " << unsigned(reg) << " bit " << unsigned(idx) << " to 0" << std::endl;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, sizeof(buffer));

	if(i2c_read(&device, reg, buffer, size) != size) {
		std::cout << "error reading from " << reg << " register" << std::endl;
	}
	buffer[0] = ~(1 << idx) & buffer[0];
	if (i2c_write(&device, reg, buffer, size) != size) {
		std::cout << "error writing to " << reg << " register" << std::endl;
	}
}

void ServoDriver::writeToRegister(uint8_t reg, uint8_t value) {
	std::cout << "writing value " << unsigned(value) << " to reg " << unsigned(reg) << std::endl;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = value;

	if (i2c_write(&device, reg, buffer, size) != size) {
		std::cout << "error writing to " << reg << " register" << std::endl;
	}
}