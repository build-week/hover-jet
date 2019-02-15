//%bin(filter_viz_balsaq_main)
#include "filtering/filter_viz_balsaq.hh"

#include "config/fiducial_map/read_fiducial_map.hh"
#include "control/servo_interface.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "filtering/pose_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "vision/fiducial_detection_message.hh"
#include "visualization/thrust_stand_visualizer.hh"

#include <cstddef>
#include <iostream>

// %deps(simple_geometry)
// %deps(window_3d)
// %deps(fit_ellipse)
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

void FilterVizBq::init(const Config& config) {
  const auto view = viewer::get_window3d("Filter Debug");
  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), jcc::Vec3(-1.0, 0.0, -1.0)));
  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground, 0.1});
  background->flip();

  sensor_geo_ = view->add_primitive<viewer::SimpleGeometry>();
  pose_geo_ = view->add_primitive<viewer::SimpleGeometry>();
  servo_geo_ = view->add_primitive<viewer::SimpleGeometry>();

  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");

  std::cout << "Subscribing pose" << std::endl;
  pose_sub_ = make_subscriber("pose");

  std::cout << "Subscribing servos" << std::endl;
  servo_sub_ = make_subscriber("servo_command_channel");

  tag_from_world_ = get_fiducial_pose().tag_from_world;
  camera_from_body_ = get_camera_extrinsics().camera_from_frame;

  std::cout << "Filter Viz starting" << std::endl;
  gonogo().go();
}

void FilterVizBq::draw_sensors() {
  const auto view = viewer::get_window3d("Filter Debug");

  ImuMessage imu_msg;
  FiducialDetectionMessage detection_msg;
  if (true && fiducial_sub_->read(detection_msg, 1)) {
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

  MatNd<3, 3> L;
  {  // clang-format off
    L.row(0) << 9.67735, 0, 0;
    L.row(1) << 0.136597, 9.59653, 0;
    L.row(2) << -0.216635, 0.00400047, 9.64812;
    // clang-format on
  }
  const jcc::Vec3 offset(0.0562102, 0.42847, -0.119841);
  const geometry::shapes::Ellipse ellipse{L, offset};

  while (imu_sub_->read(imu_msg, 1)) {
    const jcc::Vec3 accel_mpss(imu_msg.accel_mpss_x, imu_msg.accel_mpss_y, imu_msg.accel_mpss_z);
    const jcc::Vec3 mag_utesla(imu_msg.mag_utesla_x, imu_msg.mag_utesla_y, imu_msg.mag_utesla_z);
    const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y, imu_msg.gyro_radps_z);

    accel_history_.push_back(
        {geometry::shapes::deform_ellipse_to_unit_sphere(accel_mpss, ellipse) * 9.81, gyro_radps, mag_utesla});
    mag_utesla_.push_back({mag_utesla});

    std::cout << accel_mpss.transpose() << ", " << accel_mpss.norm() << std::endl;
  }

  while (accel_history_.size() > 25000u) {
    accel_history_.pop_front();
  }

  std::vector<jcc::Vec3> accels;
  accels.reserve(accel_history_.size());
  for (const auto& meas : accel_history_) {
    accels.push_back(meas.accel_mpss);
    sensor_geo_->add_point({meas.accel_mpss, jcc::Vec4(0.1, 0.7, 0.3, 0.9)});
  }

  if (accel_history_.size() > 2500u) {
    const auto visitor = [this, &view](const geometry::shapes::EllipseFit& fit) {
      sensor_geo_->add_ellipsoid({fit.ellipse, jcc::Vec4(0.4, 0.6, 0.4, 0.7), 2.0});
      sensor_geo_->flush();
    };
    // std::cout << "Optimizing" << std::endl;
    const auto result = geometry::shapes::fit_ellipse(accels);
    sensor_geo_->add_ellipsoid({result.ellipse, jcc::Vec4(0.2, 0.9, 0.2, 1.0), 5.0});

    std::cerr << "chol" << std::endl;
    std::cerr << result.ellipse.cholesky_factor << std::endl;
    std::cerr << "p0" << std::endl;
    std::cerr << result.ellipse.p0.transpose() << std::endl;
  }

  while (fiducial_history_.size() > 10u) {
    fiducial_history_.pop_front();
  }

  while (mag_utesla_.size() > 10u) {
    mag_utesla_.pop_front();
  }

  sensor_geo_->add_sphere({tag_from_world_.inverse().translation(), 0.3});
  sensor_geo_->add_axes({tag_from_world_.inverse()});

  sensor_geo_->add_sphere({camera_from_body_.inverse().translation(), 0.3});
  sensor_geo_->add_axes({camera_from_body_.inverse()});

  if (!fiducial_history_.empty()) {
    const SE3 fiducial_from_camera = fiducial_history_.back();

    constexpr bool DRAW_FIDUCIAL_POSE = false;
    constexpr bool DRAW_VEHICLE_POSE = true;
    constexpr bool DRAW_FIDUCIAL_IN_BODY_FRAME = false;

    if (DRAW_FIDUCIAL_POSE) {  // Draw the fiducial axes in the camera frame
      sensor_geo_->add_sphere({fiducial_from_camera.inverse().translation(), 0.2, jcc::Vec4(1.0, 1.0, 0.2, 0.8)});
      sensor_geo_->add_axes({fiducial_from_camera.inverse(), 0.025, 3.0});
    }

    if (DRAW_VEHICLE_POSE) {  // Draw the vehicle axes in the world frame
      const SE3 world_from_vehicle = tag_from_world_.inverse() * fiducial_from_camera * camera_from_body_;
      sensor_geo_->add_sphere({world_from_vehicle.translation(), 0.2, jcc::Vec4(1.0, 0.0, 0.2, 0.8)});
      sensor_geo_->add_axes({world_from_vehicle, 0.6, 3.0, true});
    }

    if (DRAW_FIDUCIAL_IN_BODY_FRAME) {  // Draw the body axes in the fiducial frame
      const SE3 fiducial_from_body = fiducial_from_camera * camera_from_body_;
      sensor_geo_->add_axes({fiducial_from_body.inverse()});
      sensor_geo_->add_sphere({fiducial_from_body.inverse().translation(), 0.2, jcc::Vec4(0.0, 0.0, 1.0, 0.8)});
    }
  }

  if (!accel_history_.empty()) {
    sensor_geo_->add_line({jcc::Vec3::Zero(), accel_history_.back().accel_mpss});
  }

  sensor_geo_->add_sphere({jcc::Vec3::Zero(), 9.81});
  sensor_geo_->flip();
}

void FilterVizBq::draw_pose() {
  PoseMessage pose_msg;
  bool got_pose_msg = false;
  while (pose_sub_->read(pose_msg, 1)) {
    got_pose_msg = true;
  }

  if (got_pose_msg) {
    const Pose pose = pose_msg.to_pose();
    pose_geo_->add_axes({pose.world_from_jet, 0.055, 3.0, true});
    pose_geo_->flip();
  }
}

void FilterVizBq::draw_servos() {
  const control::VaneConfiguration vane_cfg = {};
  const control::QuadraframeConfiguration qframe_cfg = {};

  SetServoMessage servo_message;
  bool got_servo_msg = false;
  while (servo_sub_->read(servo_message, 1)) {
    got_servo_msg = true;
  }
  if (got_servo_msg) {
    const control::QuadraframeStatus qframe_status = control::create_quadraframe_status(servo_message);
    visualization::put_quadraframe(*servo_geo_, qframe_status, qframe_cfg, vane_cfg);
    std::cout << "--" << std::endl;
    std::cout << qframe_status.servo_0_angle_rad << ", " << qframe_status.servo_1_angle_rad << ", "
              << qframe_status.servo_2_angle_rad << ", " << qframe_status.servo_3_angle_rad << std::endl;

    servo_geo_->flip();
  }
}

void FilterVizBq::loop() {
  draw_sensors();
  draw_pose();
  draw_servos();
}

void FilterVizBq::shutdown() {
  std::cout << "Filter Viz shutdown" << std::endl;
}

}  // namespace embedded
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::embedded::FilterVizBq)
