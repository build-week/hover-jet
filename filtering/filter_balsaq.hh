
#pragma once

#include <memory>
#include <queue>

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

//%deps(jet_filter)
#include "third_party/experiments/estimation/jet/jet_filter.hh"

namespace jet {
namespace filtering {

class FilterBq : public BalsaQ {
 public:
  FilterBq();
  void init(int argc, char *argv[]);
  void loop();
  void shutdown();

  // Every 10 millisecond
  const static uint loop_delay_microseconds = 10000;

 private:
  SubscriberPtr fiducial_sub_;
  SubscriberPtr imu_sub_;

  PublisherPtr pose_pub_;

  SE3 camera_from_vehicle_;
  SE3 tag_from_world_;
  estimation::jet_filter::JetFilter jf_;
};

}  // namespace filtering
}  // namespace jet
