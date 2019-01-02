
#pragma once

#include "embedded/imu_driver/imu_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {
namespace embedded {

class ImuBq : public BalsaQ {
 public:
  ImuBq();
  void init();
  void loop();
  void shutdown();

  // Every millisecond
  const static uint loop_delay_microseconds = 1000;

 private:
  PublisherPtr publisher_;
  ImuDriver imu_driver;
};

}  // namespace embedded
}  // namespace jet
