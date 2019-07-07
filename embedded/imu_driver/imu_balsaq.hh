#pragma once

#include "embedded/imu_driver/imu_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {
namespace embedded {

struct ManagedDriver {
  ImuDriver driver;
  PublisherPtr publisher;
  int imu_unique_identifier;

  uint8_t i2c_address;
  std::string i2c_bus;

  bool reset() {
    return driver.initialize(i2c_bus, i2c_address);
  }
};

class ImuBq : public BalsaQ {
 public:
  ImuBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

  // Every 10 milliseconds
  const static uint loop_delay_microseconds = 10000;

 private:
  std::vector<ManagedDriver> imu_drivers_;
};

}  // namespace embedded
}  // namespace jet
