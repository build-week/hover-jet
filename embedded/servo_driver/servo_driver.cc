/***
Servo driver library for Jet hover project
@author Jasbir Harnal jaz.jlh@gmail.com

Borrows from:
Adafruit PCA9685 library: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/
Other help:

***/

#include "servo_driver.hh"

// This is for the PCA9685
#define PCA9685_I2C_ADDR 0x40
#define I2C_SLAVE 0x0703
#define I2C_SLAVE_FORCE 0x0706

// These constants are straight from Adafruit's Library
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


ServoDriver::ServoDriver(char* dev) {
	device_name = dev;
}

uint8_t ServoDriver::init() {
	int i2cbus = i2c_open(device_name);
	if (i2cbus == -1) {
		// we could not open the interface
		std::cout << "could not open interface " << device_name << ", error: " << std::strerror(errno) << std::endl;
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
	set_pwm_freq(100);
	return 0;
}

void ServoDriver::reset() {
	set_register_bit(PCA9685_MODE1, 7);
	usleep(10000);
}

// @brief Sets the PWM frequency for the chip - max ~1.6kHz
void ServoDriver::set_pwm_freq(float freq) {
	std::cout << "setting PWM frequency to " << freq << std::endl;
	freq *= 0.95; // this is necessary according to the Adafruit lib
	// this formula is on page 25 of the PCA9685 datasheet
	float prescaleval = 25000000; // 25Mhz oscillator clock freq
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	uint8_t prescale = floor(prescaleval + 0.5);

	// We need to go into sleep mode before setting the prescaler
	set_register_bit(PCA9685_MODE1, 4);					//go to sleep
	write_to_register(PCA9685_PRESCALE, prescale);		//write prescale to register
	clear_register_bit(PCA9685_MODE1, 4);				    //bring it out of sleep mode
	usleep(5000);
	enable_auto_increment(true);							//set the mode to auto increment
}

//@brief Sets the PWM output of one of the PCA9685 pins
void ServoDriver::set_pwm(uint8_t servo_num, uint16_t start, uint16_t stop) {
	std::cout << "setting PWM from " << start << "-" << stop << std::endl;
	//first we want to clamp the values between 0-4095
	start = std::min(start, (uint16_t)4095);
	stop = std::min(stop, (uint16_t)4095);
	// base address is LED0_ON_L, each servo has a 4 byte register size
	// calculate the address based on the servo number
	uint8_t base_reg = LED0_ON_L + 4*servo_num;

	// fill in the registers
	write_to_register(base_reg, start);
	write_to_register(base_reg+1, start >> 8);
	write_to_register(base_reg+2, stop);
	write_to_register(base_reg+3, stop >> 8);
}

void ServoDriver::enable_auto_increment(bool enable) {
	if (enable) {
		set_register_bit(PCA9685_MODE1, 5);
	}
	else {
		clear_register_bit(PCA9685_MODE1, 5);
	}
}

void ServoDriver::set_register_bit(uint8_t reg, uint8_t idx) {
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

void ServoDriver::clear_register_bit(uint8_t reg, uint8_t idx) {
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

void ServoDriver::write_to_register(uint8_t reg, uint8_t value) {
	std::cout << "writing value " << unsigned(value) << " to reg " << unsigned(reg) << std::endl;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = value;

	if (i2c_write(&device, reg, buffer, size) != size) {
		std::cout << "error writing to " << reg << " register" << std::endl;
	}
}