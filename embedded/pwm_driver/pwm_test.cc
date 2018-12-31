#include "pwm_driver.hh"
#include <iostream>
#include <unistd.h> // usleep

int main(void) {
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
    return -1;
  }
  PwmDriver driver = PwmDriver(i2cHandle);
  driver.set_pwm_freq(50.0);
  driver.enable_auto_increment(true);
  const uint8_t channel = 1;
  while (1) {
    for (int i = 0; i < 4096; i++) {
      driver.set_pwm(channel, 0, i);
      usleep(1000);
    }
  }
  return 0;
}
