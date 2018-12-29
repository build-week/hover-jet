/***
Pwm driver library for Jet hover project
@author Jasbir Harnal jaz.jlh@gmail.com

Borrows from:
Adafruit PCA9685 library:
https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/

***/

#include "embedded/pwm_driver/pwm_driver.hh" // driver library
#include <algorithm>                         // min
#include <cmath>                             // floor
#include <iostream>
#include <string.h> // memset
#include <unistd.h> // usleep

// This is for the PCA9685
#define PCA9685_I2C_ADDR 0x42
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

PwmDriver::PwmDriver(const std::string &dev) {
  const char *device_name = dev.c_str();
  int i2cbus = i2c_open(device_name);
  if (i2cbus == -1) {
    throw std::runtime_error(std::string("Could not open i2c"));
    return;
  }
  // allocate memory for the device struct
  memset(&device, 0, sizeof(device));

  device.bus = i2cbus;
  device.addr = PCA9685_I2C_ADDR;
  device.iaddr_bytes = 1; // size of internal addresses = 1 byte
  device.page_bytes = 16; // device is capapble of 16 bytes per page

  // TODO check if pwm driver can be found
  // first we want to reset the device
  reset();
  // then set a default frequency
  set_pwm_freq(100);
}

void PwmDriver::reset() {
  set_register_bit(PCA9685_MODE1, 7);
  usleep(10000);
}

// @brief Sets the PWM frequency for the chip - max ~1.6kHz
void PwmDriver::set_pwm_freq(float freq) {
  freq *= 0.95; // this is necessary according to the Adafruit lib
  // this formula is on page 25 of the PCA9685 datasheet
  const float osc_clock = 25000000; // 25Mhz oscillator clock freq
  const float prescaleval_scaled_by_max = osc_clock / 4096;
  const float prescaleval_scaled_by_freq = prescaleval_scaled_by_max / freq;
  const float prescaleval = prescaleval_scaled_by_freq - 1;
  uint8_t prescale = std::floor(prescaleval + 0.5);

  // We need to go into sleep mode before setting the prescaler
  set_register_bit(PCA9685_MODE1, 4);            // go to sleep
  write_to_register(PCA9685_PRESCALE, prescale); // write prescale to
                                                 // register
  clear_register_bit(PCA9685_MODE1, 4);          // bring it out of sleep mode
  usleep(5000);
  enable_auto_increment(true); // set the mode to auto increment
}

//@brief Sets the PWM output of one of the PCA9685 pins
void PwmDriver::set_pwm(uint8_t pwm_num, uint16_t start, uint16_t stop) {
  // first we want to clamp the values between 0-4095
  start = std::min(start, (uint16_t)4095);
  stop = std::min(stop, (uint16_t)4095);
  // base address is LED0_ON_L, each servo has a 4 byte register size
  // calculate the address based on the servo number
  uint8_t base_reg = LED0_ON_L + 4 * pwm_num;

  // fill in the registers
  write_to_register(base_reg, start);
  write_to_register(base_reg + 1, start >> 8);
  write_to_register(base_reg + 2, stop);
  write_to_register(base_reg + 3, stop >> 8);
}

void PwmDriver::enable_auto_increment(bool enable) {
  if (enable) {
    set_register_bit(PCA9685_MODE1, 5);
  } else {
    clear_register_bit(PCA9685_MODE1, 5);
  }
}

void PwmDriver::set_register_bit(uint8_t reg, uint8_t idx) {
  unsigned char buffer[1];
  ssize_t size = sizeof(buffer);
  memset(buffer, 0, sizeof(buffer));

  i2c_read(&device, reg, buffer, size);
  buffer[0] = (1 << idx) | buffer[0];
  i2c_write(&device, reg, buffer, size);
}

void PwmDriver::clear_register_bit(uint8_t reg, uint8_t idx) {
  unsigned char buffer[1];
  ssize_t size = sizeof(buffer);
  memset(buffer, 0, sizeof(buffer));

  i2c_read(&device, reg, buffer, size);
  buffer[0] = ~(1 << idx) & buffer[0];
  i2c_write(&device, reg, buffer, size);
}

void PwmDriver::write_to_register(uint8_t reg, uint8_t value) {
  unsigned char buffer[1];
  ssize_t size = sizeof(buffer);
  memset(buffer, 0, sizeof(buffer));
  buffer[0] = value;

  i2c_write(&device, reg, buffer, size);
}
