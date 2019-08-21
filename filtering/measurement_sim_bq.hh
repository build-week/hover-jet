
#pragma once

#include <memory>
#include <queue>

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "vision/fiducial_detection_message.hh"

#include "filtering/imu_model_from_yaml.hh"

#include "filtering/transform_network_from_yaml.hh"

namespace jet {
namespace filtering {

class MeasurementSimBq : public BalsaQ {
 public:
  MeasurementSimBq();
  void init(const Config& config);
  void loop();
  void shutdown();

  // Every 10 millisecond
  const static uint loop_delay_microseconds = 10000;

 private:
  double fiducial_latency_s_ = 0.15;

  std::map<int, estimation::ImuModel> imu_model_from_id_;
  geometry::TransformNetwork transform_network_;

  PublisherPtr imu_1_pub_;
  PublisherPtr imu_2_pub_;
  PublisherPtr fiducial_pub_;
  Timestamp last_publish_{};
};

}  // namespace filtering
}  // namespace jet
