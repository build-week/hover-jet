//%bin(filter_viz_balsaq_main)
#include "filtering/filter_viz_balsaq.hh"

#include "config/fiducial_map/read_fiducial_map.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "filtering/pose_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "vision/fiducial_detection_message.hh"

#include <cstddef>
#include <iostream>

// %deps(simple_geometry)
// %deps(window_3d)
//%deps(fit_ellipse)
#include "third_party/experiments/estimation/time_point.hh"
#include "third_party/experiments/geometry/shapes/fit_ellipse.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

namespace jet {
namespace embedded {

FilterVizBq::FilterVizBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

estimation::TimePoint to_time_point(const Timestamp& ts) {
  const auto epoch_offset = std::chrono::nanoseconds(uint64_t(ts));
  const estimation::TimePoint time_point = estimation::TimePoint{} + epoch_offset;
  return time_point;
}

void FilterVizBq::init(int argc, char* argv[]) {
  const auto view = viewer::get_window3d("Filter Debug");
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), jcc::Vec3(-1.0, 0.0, -1.0)));
  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground, 0.1});
  background->flip();

  geo_ = view->add_primitive<viewer::SimpleGeometry>();
  persistent_ = view->add_primitive<viewer::SimpleGeometry>();

  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");

  std::cout << "Subscribing pose" << std::endl;
  pose_sub_ = make_subscriber("pose");

  tag_from_world_ = get_fiducial_pose().tag_from_world;
  camera_from_body_ = get_camera_extrinsics().camera_from_frame.inverse();

  std::cout << "Filter starting" << std::endl;
}

void FilterVizBq::draw_sensors() {
  const auto view = viewer::get_window3d("Filter Debug");

  ImuMessage imu_msg;
  FiducialDetectionMessage detection_msg;
  if (fiducial_sub_->read(detection_msg, 1)) {
    const SE3 fiducial_from_camera = detection_msg.fiducial_from_camera();
    const SE3 fiducial_from_body = fiducial_from_camera * camera_from_body_;

    // This is used to generate a config
    const jcc::Vec3 log_translation_tag_from_world = fiducial_from_body.translation();
    const jcc::Vec3 log_rotation_tag_from_world = fiducial_from_body.so3().log();
    std::cout << "log_translation_tag_from_world: [" << log_translation_tag_from_world[0] << ", "
              << log_translation_tag_from_world[1] << ", " << log_translation_tag_from_world[2] << "] " << std::endl;

    std::cout << "log_rotation_tag_from_world: [" << log_rotation_tag_from_world[0] << ", "
              << log_rotation_tag_from_world[1] << ", " << log_rotation_tag_from_world[2] << "] " << std::endl;

    const auto fiducial_time_of_validity = to_time_point(detection_msg.timestamp);

    fiducial_history_.push_back(fiducial_from_camera);
  }

  while (imu_sub_->read(imu_msg, 1)) {
    const jcc::Vec3 accel_mpss(imu_msg.accel_mpss_x, imu_msg.accel_mpss_y, imu_msg.accel_mpss_z);
    const jcc::Vec3 mag_utesla(imu_msg.mag_utesla_x, imu_msg.mag_utesla_y, imu_msg.mag_utesla_z);
    const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y, imu_msg.gyro_radps_z);

    accel_history_.push_back({accel_mpss, gyro_radps, mag_utesla});
    mag_utesla_.push_back({mag_utesla});
  }

  while (accel_history_.size() > 15u) {
    accel_history_.pop_front();
  }

  while (fiducial_history_.size() > 10u) {
    fiducial_history_.pop_front();
  }

  while (mag_utesla_.size() > 10u) {
    mag_utesla_.pop_front();
  }

  geo_->add_sphere({tag_from_world_.inverse().translation(), 0.3});
  geo_->add_axes({tag_from_world_.inverse()});

  if (!fiducial_history_.empty()) {
    const SE3 fiducial_from_camera = fiducial_history_.back();
    geo_->add_axes({fiducial_from_camera, 0.025, 3.0});

    const SE3 world_from_vehicle = tag_from_world_.inverse() * fiducial_from_camera * camera_from_body_;
    geo_->add_sphere({world_from_vehicle.translation(), 0.2, jcc::Vec4(1.0, 0.0, 0.2, 0.8)});
    geo_->add_axes({world_from_vehicle, 0.6, 3.0, true});
  }

  if (!accel_history_.empty()) {
    geo_->add_line({jcc::Vec3::Zero(), accel_history_.back().accel_mpss});
  }

  geo_->add_sphere({jcc::Vec3::Zero(), 9.81});
}

void FilterVizBq::draw_pose() {
  PoseMessage pose_msg;
  bool got_pose_msg = false;
  while (pose_sub_->read(pose_msg, 1)) {
    got_pose_msg = true;
  }

  if (got_pose_msg) {
    const Pose pose = pose_msg.to_pose();
    std::cout << "Got pose message" << std::endl;
    geo_->add_axes({pose.world_from_jet, 0.055, 3.0, true});
  }
}

void FilterVizBq::loop() {
  draw_sensors();
  draw_pose();

  geo_->flip();
}

void FilterVizBq::shutdown() {
  std::cout << "Filter Viz shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::FilterVizBq)
