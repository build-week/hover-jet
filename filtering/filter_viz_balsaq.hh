
#pragma once

#include <memory>
#include <queue>

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

//%deps(jet_filter)
#include "third_party/experiments/estimation/jet/jet_filter.hh"

#include "third_party/experiments/viewer/primitives/simple_geometry.hh"

namespace jet {
namespace embedded {

class FilterVizBq : public BalsaQ {
 public:
  FilterVizBq();
  void init(const Config& config);
  void loop();
  void shutdown();

  void draw_sensors();
  void draw_pose();
  void draw_servos();

  // Every millisecond
  const static uint loop_delay_microseconds = 10000;

 private:
  SubscriberPtr fiducial_sub_;
  SubscriberPtr imu_sub_;
  SubscriberPtr pose_sub_;
  SubscriberPtr servo_sub_;

  SE3 tag_from_world_;
  SE3 camera_from_body_;

  std::shared_ptr<viewer::SimpleGeometry> sensor_geo_;
  std::shared_ptr<viewer::SimpleGeometry> pose_geo_;
  std::shared_ptr<viewer::SimpleGeometry> servo_geo_;

  struct AccelStuff {
    jcc::Vec3 accel_mpss;
    jcc::Vec3 gyro_radps;
    jcc::Vec3 mag_utesla;
  };

  std::deque<AccelStuff> accel_history_;
  std::deque<SE3> fiducial_history_;
  std::deque<jcc::Vec3> mag_utesla_;

  estimation::jet_filter::JetFilter jf_;
};

}  // namespace embedded
}  // namespace jet
