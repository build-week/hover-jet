//%deps(ina219)

#include "third_party/ina219/Adafruit_INA219.hh"
#include <iostream>
#include <unistd.h> // usleep

int main() {
    Adafruit_INA219 ina219 = Adafruit_INA219("/dev/i2c-1", 0x40);
    ina219.begin();
    std::cout << "Current Sensor Test" << std::endl;
    while (true) {
        usleep(100 * 1000);
        float shuntvoltage = 0;
        float busvoltage = 0;
        float current_mA = 0;
        float loadvoltage = 0;
        float power_mW = 0;

        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        loadvoltage = busvoltage + (shuntvoltage / 1000);

        std::cout << "Bus Voltage: " << busvoltage << std::endl;
        std::cout << "Shunt Voltage: " << shuntvoltage << std::endl;
        std::cout << "Load Voltage: " << loadvoltage << std::endl;
        std::cout << "Current: " << current_mA << "mA" << std::endl;
        std::cout << "Power: " << power_mW << "mW" <<  std::endl;
        std::cout << std::endl;
    }
}
