//%deps(ina219)

#include <unistd.h>  // usleep
#include <iostream>
#include "third_party/ina219/Adafruit_INA219.hh"

int main() {
  constexpr int i2c_addr = 0x40;
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
    return -1;
  }

  Adafruit_INA219 ina219 = Adafruit_INA219(i2cHandle, i2c_addr);
  std::cout << "Current Sensor Test" << std::endl;
  while (true) {
    usleep(100 * 1000);

    float shuntvoltage = ina219.getShuntVoltage_mV();
    float busvoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
    float loadvoltage = busvoltage + (shuntvoltage / 1000);

    std::cout << "Bus Voltage: " << busvoltage << std::endl;
    std::cout << "Shunt Voltage: " << shuntvoltage << std::endl;
    std::cout << "Load Voltage: " << loadvoltage << std::endl;
    std::cout << "Current: " << current_mA << "mA" << std::endl;
    std::cout << "Power: " << power_mW << "mW" << std::endl;
    std::cout << std::endl;
  }
}
