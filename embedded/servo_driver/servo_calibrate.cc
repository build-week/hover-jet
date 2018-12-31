//%deps(readline)
//%deps(ina219)
#include <readline/readline.h>
#include <unistd.h>  // usleep
#include <iostream>
#include <vector>
#include "servo_driver.hh"
#include "third_party/ina219/Adafruit_INA219.hh"

constexpr int PWM_FREQUENCY = 330;

// Make these global variables for rl_bind_key to work.
std::vector<ServoDriver> servos;
Adafruit_INA219 *ina219;

enum Direction { UP, DOWN };

void clear_screen() {
  std::cout << "\033[2J" << std::endl;
  std::cout << "\033[2H" << std::endl;
}

void print_help(const std::vector<ServoDriver> &servos) {
  std::cout << "Servo 0: Press '1' for up, '2' for down -- current percentage is "
            << servos[0].get_percentage() << std::endl;
  std::cout << "Servo 1: Press '3' for up, '4' for down -- current percentage is "
            << servos[1].get_percentage() << std::endl;
  std::cout << "Servo 2: Press '5' for up, '6' for down -- current percentage is "
            << servos[2].get_percentage() << std::endl;
  std::cout << "Servo 3: Press '7' for up, '8' for down -- current percentage is "
            << servos[3].get_percentage() << std::endl;
  std::cout << std::endl;

  std::cout << "Press Enter to start reading currents from the current sensor"
            << std::endl;
}

int keypress(int count, int key) {
  clear_screen();
  int num = key - '0';
  ServoDriver &servo = servos[(num - 1) / 2];
  Direction direction = num % 2 == 1 ? UP : DOWN;
  if (direction == DOWN) {
    servo.set_percentage(std::max(0, servo.get_percentage() - 1));
  } else {
    servo.set_percentage(std::min(100, servo.get_percentage() + 1));
  }
  print_help(servos);
  return 0;
}

int main() {
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
    return -1;
  }
  clear_screen();
  std::shared_ptr<PwmDriver> pwm_driver = std::make_shared<PwmDriver>(i2cHandle);
  pwm_driver->set_pwm_freq(PWM_FREQUENCY);
  pwm_driver->enable_auto_increment(true);

  ServoDriver servo1 =
      ServoDriver(0, pwm_driver, "../embedded/servo_driver/cfg/servo_cfg0.yaml");
  ServoDriver servo2 =
      ServoDriver(1, pwm_driver, "../embedded/servo_driver/cfg/servo_cfg1.yaml");
  ServoDriver servo3 =
      ServoDriver(2, pwm_driver, "../embedded/servo_driver/cfg/servo_cfg2.yaml");
  ServoDriver servo4 =
      ServoDriver(3, pwm_driver, "../embedded/servo_driver/cfg/servo_cfg3.yaml");

  servos.push_back(servo1);
  servos.push_back(servo2);
  servos.push_back(servo3);
  servos.push_back(servo4);

  constexpr int CURRENT_SENSOR_I2C_ADDR = 0x40;
  Adafruit_INA219 ina219Local = Adafruit_INA219(i2cHandle, CURRENT_SENSOR_I2C_ADDR);
  ina219 = &ina219Local;

  for (int i = 0; i < 10; i++) {
    rl_bind_key('0' + i, keypress);
  }

  print_help(servos);
  readline("");
  while (true) {
    float current_mA = ina219->getCurrent_mA();
    std::cout << "Current reading: " << (int)current_mA << std::endl;
    usleep(10 * 100);
  }
  return 0;
}
