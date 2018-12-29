//%deps(ina219)

#include "third_party/ina219/Adafruit_INA219.hh"
#include <iostream>
#include <unistd.h> // usleep

int main() {
    constexpr int i2c_addr = 0x40;
    Adafruit_INA219 ina219 = Adafruit_INA219("/dev/i2c-1", i2c_addr);
    ina219.begin();
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
        std::cout << "Power: " << power_mW << "mW" <<  std::endl;
        std::cout << std::endl;
    }
}
