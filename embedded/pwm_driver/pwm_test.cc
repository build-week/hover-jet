#include "pwm_driver.hh"
#include <iostream>
#include <unistd.h> // usleep

int main(void) {
  PwmDriver driver = PwmDriver("/dev/i2c-1");
  driver.init();
  driver.set_pwm_freq(50.0);
  driver.enable_auto_increment(true);
  const uint8_t channel = 1;
  while(1) {
    for (int i=0; i<4096; i++) {
      driver.set_pwm(channel, 0, i);
      usleep(1000);
    }
  }
  return 0;
}
