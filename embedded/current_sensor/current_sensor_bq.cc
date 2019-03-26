//%bin(current_sensor_bq_main)
//%deps(balsa_queue)
//%deps(ina219)

#include "embedded/current_sensor/current_sensor_bq.hh"
#include "embedded/current_sensor/power_reading.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <iostream>

namespace jet {

void CurrentSensorBq::init(int argc, char *argv[]) {
  constexpr int i2c_addr = 0x40;
  int i2cHandle = i2c_open("/dev/i2c-1");
  if (i2cHandle == -1) {
    std::cout << "Failed to open i2c" << std::endl;
  }

  sensor_ptr_ = std::make_unique<Adafruit_INA219>(i2cHandle, i2c_addr);
  power_publisher_ = make_publisher("current_sensor_sensor_id");
}

void CurrentSensorBq::loop() {
  PowerReading power_reading_message;
  power_reading_message.bus_voltage_V = sensor_ptr_->getBusVoltage_V();
  power_reading_message.current_mA = sensor_ptr_->getCurrent_mA();
  power_reading_message.power_mW = sensor_ptr_->getPower_mW();
  power_publisher_->publish(power_reading_message);
}

void CurrentSensorBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::CurrentSensorBq)
