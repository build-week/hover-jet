
#pragma once

#include "embedded/imu_driver/imu_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {
namespace embedded {

class ImuBq : public BalsaQ {
 public:
  ImuBq() = default;
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

  // Every millisecond
  const static uint loop_delay_microseconds = 1000;

 private:
  PublisherPtr publisher_;
  ImuDriver imu_driver_;
};

}  // namespace embedded
}  // namespace jet
