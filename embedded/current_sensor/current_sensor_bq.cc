//%bin(current_sensor_bq_main)
//%deps(balsa_queue)
//%deps(message)

#include "embedded/current_sensor/current_sensor_bq.hh"
#include "embedded/current_sensor/power_reading.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <iostream>

namespace jet {

void CurrentSensorBq::init(const Config& config) {
  assert(config["i2c_bus_path"]);
  assert(config["i2c_address"]);
  assert(config["sensor_name"]);

  const int i2c_addr = config["i2c_address"].as<int>();
  int i2c_handle = i2c_open(config["i2c_bus_path"].as<std::string>().c_str());
  if (i2c_handle == -1) {
    std::cerr << "Failed to open i2c" << std::endl;
    exit(1);
  }

  sensor_ptr_ =
      std::make_unique<ina219::INA219Driver>(i2c_handle, i2c_addr, ina219::DriverConfiguration::make_32V_2A());
  power_publisher_ = make_publisher(std::string("current_sensor_sensor") + config["sensor_name"].as<std::string>());
}

void CurrentSensorBq::loop() {
  PowerReading power_reading_message;
  if (const auto shunt_voltage_mV = sensor_ptr_->get_shunt_voltage_mV()) {
    power_reading_message.bus_voltage_mV = shunt_voltage_mV.value();
  }
  if (const auto shunt_voltage_mV = sensor_ptr_->get_current_mA()) {
    power_reading_message.current_mA = shunt_voltage_mV.value();
  }
  if (const auto shunt_voltage_mV = sensor_ptr_->get_power_mW()) {
    power_reading_message.power_mW = shunt_voltage_mV.value();
  }
  power_publisher_->publish(power_reading_message);
}

void CurrentSensorBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CurrentSensorBq)
