
#pragma once

#include <memory>

#include "embedded/imu_driver/imu_driver.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "third_party/experiments/viewer/primitives/simple_geometry.hh"

namespace jet {
namespace embedded {

class FilterBq : public BalsaQ {
 public:
  FilterBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

  // Every millisecond
  const static uint loop_delay_microseconds = 1000;

 private:
  SubscriberPtr fiducial_sub_;
  SubscriberPtr imu_sub_;
  ImuDriver imu_driver_;

  std::shared_ptr<viewer::SimpleGeometry> geo_;
  std::shared_ptr<viewer::SimpleGeometry> persistent_;
};

}  // namespace embedded
}  // namespace jet
