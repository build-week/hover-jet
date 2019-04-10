#pragma once

#include <memory>
#include <queue>

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

namespace jet {
namespace simulation {

class PoseSimulatorBq : public BalsaQ {
 public:
  PoseSimulatorBq();
  void init(const Config& config);
  void loop();
  void shutdown();

  // Every 10 millisecond
  const static uint loop_delay_microseconds = 10000;

 private:
  PublisherPtr pose_pub_;
  SubscriberPtr imu_sub_;
};

}  // namespace simulation
}  // namespace jet
