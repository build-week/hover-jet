
#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

//%deps(jet_filter_manager)
#include "third_party/experiments/estimation/jet/jet_filter_manager.hh"

namespace jet {
namespace filtering {

namespace ejf = estimation::jet_filter;
class FilterBq : public BalsaQ {
 public:
  FilterBq();
  void init(const Config& config);
  void loop();
  void shutdown();

  // Every 10 millisecond
  const static uint loop_delay_microseconds = 10000;

 private:
  SubscriberPtr fiducial_sub_;

  std::map<int, SubscriberPtr> imu_sub_from_id_;

  ejf::FilterManager filter_manager_;

  PublisherPtr pose_pub_;
  PublisherPtr state_pub_;
};

}  // namespace filtering
}  // namespace jet
