//%bin(filter_balsaq_main)
#include "filtering/filter_balsaq.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "config/fiducial_map/read_fiducial_map.hh"
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

void FilterBq::init(const Config& config) {
  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");
  std::cout << "Advertising Pose" << std::endl;
  pose_pub_ = make_publisher("pose");

  camera_from_vehicle_ = get_camera_extrinsics().camera_from_frame;
  tag_from_world_ = get_fiducial_pose().tag_from_world;

  std::cout << "Filter starting" << std::endl;
}

void FilterBq::loop() {
  FiducialDetectionMessage detection_msg;

  //
  // Update with fiducial measurements
  //
  if (fiducial_sub_->read(detection_msg, 1)) {
    estimation::jet_filter::FiducialMeasurement fiducial_meas;
    const SE3 fiducial_from_camera = detection_msg.fiducial_from_camera();
    const SE3 world_from_camera = tag_from_world_.inverse() * fiducial_from_camera;
    fiducial_meas.T_fiducial_from_camera = world_from_camera * camera_from_vehicle_;

    const auto fiducial_time_of_validity = to_time_point(detection_msg.timestamp);

    if (!jf_.initialized()) {
      std::cout << "Initializing" << std::endl;
      auto xp0 = estimation::jet_filter::JetFilter::reasonable_initial_state();
      xp0.x.R_world_from_body = fiducial_meas.T_fiducial_from_camera.so3();
      xp0.x.x_world = fiducial_meas.T_fiducial_from_camera.translation();
      xp0.time_of_validity = fiducial_time_of_validity;
      jf_.reset(xp0);

      gonogo().go();
    }

    jf_.measure_fiducial(fiducial_meas, fiducial_time_of_validity);
    const auto state = jf_.state().x;
  }

  //
  // Update with IMU measurements
  //
  ImuMessage imu_msg;
  bool got_imu_msg = false;
  while (imu_sub_->read(imu_msg, 1)) {

    got_imu_msg = true;
  }

  // if (jf_.initialized() && got_imu_msg) {
  //   const auto time_of_validity = to_time_point(imu_msg.timestamp);
  //   const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y, imu_msg.gyro_radps_z);
  //   estimation::jet_filter::GyroMeasurement gyro_meas;
  //   gyro_meas.observed_w = gyro_radps;
  //   jf_.measure_gyro(gyro_meas, time_of_validity);
  // }

  if (jf_.initialized()) {
    jf_.free_run();
  }

  //
  // Report a state
  //
  if (jf_.initialized() && got_imu_msg) {
    const auto current_time = to_time_point(imu_msg.timestamp);
    const auto state = jf_.view(current_time);

    Pose pose;
    {
      const SE3 vehicle_real_from_vehicle_filtered;
      const SE3 T_world_from_body = estimation::jet_filter::get_world_from_body(state);
      pose.world_from_jet = T_world_from_body * vehicle_real_from_vehicle_filtered.inverse();

      const jcc::Vec3 log_translation_world_from_vehicle = pose.world_from_jet.translation();
      const jcc::Vec3 log_rotation_world_from_vehicle = pose.world_from_jet.so3().log();

      pose.v_world_frame = (vehicle_real_from_vehicle_filtered.so3() * state.R_world_from_body.inverse()).inverse() *
                           state.eps_dot.head<3>();
      pose.w_world_frame = (vehicle_real_from_vehicle_filtered.so3() * state.R_world_from_body.inverse()).inverse() *
                           state.eps_dot.tail<3>();
      pose.timestamp = imu_msg.timestamp;
    }

    PoseMessage pose_msg = PoseMessage::from_pose(pose);
    pose_pub_->publish(pose_msg);
  } else {
    std::cout << "No means to queue time" << std::endl;
  }
}

void FilterBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace filtering
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::filtering::FilterBq)
