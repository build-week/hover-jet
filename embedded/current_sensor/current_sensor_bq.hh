#pragma once

#include "embedded/current_sensor/ina219_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"

#include <memory>

namespace jet {

class CurrentSensorBq : public BalsaQ {
 public:
  CurrentSensorBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  PublisherPtr power_publisher_;
  std::unique_ptr<ina219::INA219Driver> sensor_ptr_;
};

}  // namespace jet
