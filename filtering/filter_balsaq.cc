//%bin(filter_balsaq_main)
#include "filtering/filter_balsaq.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "filtering/pose_message.hh"
#include "vision/fiducial_detection_message.hh"

#include <cstddef>
#include <iostream>

// Filter requires this
#include "third_party/experiments/estimation/time_point.hh"

//%deps(fit_ellipse)
#include "third_party/experiments/geometry/shapes/fit_ellipse.hh"

namespace jet {
namespace filtering {

FilterBq::FilterBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

estimation::TimePoint to_time_point(const Timestamp& ts) {
  const auto epoch_offset = std::chrono::nanoseconds(uint64_t(ts));
  const estimation::TimePoint time_point = estimation::TimePoint{} + epoch_offset;
  return time_point;
}

void FilterBq::init(int argc, char* argv[]) {
  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");
  std::cout << "Advertising Pose" << std::endl;
  pose_pub_ = make_publisher("pose");

  std::cout << "Filter starting" << std::endl;
}

void FilterBq::loop() {
  FiducialDetectionMessage detection_msg;
  if (fiducial_sub_->read(detection_msg, 1)) {
    estimation::jet_filter::FiducialMeasurement fiducial_meas;
    fiducial_meas.T_fiducial_from_camera = detection_msg.fiducial_from_camera();

    const auto fiducial_time_of_validity = to_time_point(detection_msg.timestamp);

    if (!jf_.initialized()) {
      std::cout << "Initializing" << std::endl;
      auto xp0 = estimation::jet_filter::JetFilter::reasonable_initial_state();
      xp0.x.T_body_from_world = fiducial_meas.T_fiducial_from_camera.inverse();
      xp0.time_of_validity = fiducial_time_of_validity;
      jf_.reset(xp0);
    }

    jf_.measure_fiducial(fiducial_meas, fiducial_time_of_validity);
    jf_.free_run();
    const auto state = jf_.state().x;
  }

  ImuMessage imu_msg;
  while (imu_sub_->read(imu_msg, 1)) {
  }

  if (jf_.initialized()) {
    const auto current_time = to_time_point(imu_msg.timestamp);
    const auto state = jf_.view(current_time);

    std::cout << "Distance: " << state.T_body_from_world.inverse().translation().norm() << std::endl;
    std::cout << "eps_dot: " << state.eps_dot.transpose() << std::endl;

    Pose pose;
    {
      pose.world_from_jet = state.T_body_from_world.inverse();
      pose.v_world_frame = state.T_body_from_world.so3().inverse() * state.eps_dot.head<3>();
      pose.w_world_frame = state.T_body_from_world.so3().inverse() * state.eps_dot.tail<3>();
      pose.timestamp = imu_msg.timestamp;
    }

    PoseMessage pose_msg = PoseMessage::from_pose(pose);
    pose_pub_->publish(pose_msg);
  }
}

void FilterBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace filtering
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::filtering::FilterBq)
