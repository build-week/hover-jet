#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>

#include <third_party/ina219/Adafruit_INA219.hh>

namespace jet {

class CurrentSensorBq : public BalsaQ {
 public:
  CurrentSensorBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

 private:
  PublisherPtr power_publisher_;
  std::unique_ptr<Adafruit_INA219> sensor_ptr_;
};

}  // namespace jet
