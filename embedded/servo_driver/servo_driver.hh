#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h> // uint8_t etc
#include <unistd.h> // usleep
#include <fcntl.h> // open device
#include <math.h> // floor
#include <algorithm> // min
#include <sys/ioctl.h> // ioctl
#include <iostream> // printing
#include <errno.h> //errors
#include <cstring> //strerror
#include "i2c.h"


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

class ServoDriver {
public:
	ServoDriver(char* dev);
	uint8_t init();
	void reset(void);
	void setPWMFreq(float freq);
	void setPWM(uint8_t servo_num, uint16_t start, uint16_t stop);
	void setPin(uint8_t servo_num, uint16_t val);
	void enableAutoIncrement(bool enable);
private:
	char* deviceName;
	I2CDevice device;
	void startTransmission(bool force=false);
	void setRegisterBit(uint8_t reg, uint8_t idx);
	void clearRegisterBit(uint8_t reg, uint8_t idx);
	void writeToRegister(uint8_t reg, uint8_t value);
};

#endif
