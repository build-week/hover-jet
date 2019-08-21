//%bin(filter_viz_balsaq_main)
#include "filtering/filter_viz_balsaq.hh"

#include "config/fiducial_map/read_fiducial_map.hh"
#include "control/servo_interface.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "embedded/servo_bq/set_servo_message.hh"
#include "filtering/transform_network_from_yaml.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "vision/fiducial_detection_message.hh"
#include "visualization/thrust_stand_visualizer.hh"

// %deps(simple_geometry)
// %deps(window_3d)
// %deps(fit_ellipse)
#include "third_party/experiments/estimation/time_point.hh"
#include "third_party/experiments/geometry/shapes/fit_ellipse.hh"
#include "third_party/experiments/viewer/primitives/simple_geometry.hh"
#include "third_party/experiments/viewer/window_3d.hh"

// %deps(put_transform_network)
#include "third_party/experiments/geometry/visualization/put_transform_network.hh"

//%deps(jet_model)
#include "third_party/experiments/planning/jet/jet_model.hh"

#include <cstddef>
#include <iostream>

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

  transform_network_ = transform_network_from_yaml(config["transforms"]);

  sensor_geo_ = view->add_primitive<viewer::SimpleGeometry>();
  pose_geo_ = view->add_primitive<viewer::SimpleGeometry>();
  servo_geo_ = view->add_primitive<viewer::SimpleGeometry>();
  jet_tree_ = view->add_primitive<viewer::SceneTree>();

  const planning::jet::JetModel jet_3dmodel;
  constexpr bool DRAW_VEHICLE = true;
  if constexpr (DRAW_VEHICLE) {
    jet_3dmodel.insert(*jet_tree_);
  }

  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu_1");
  std::cout << "Subscribing Fiducial" << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");

  std::cout << "Subscribing pose" << std::endl;
  pose_sub_ = make_subscriber("pose");

  std::cout << "Subscribing servos" << std::endl;
  servo_sub_ = make_subscriber("servo_command_channel");
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

    fiducial_history_.push_back(fiducial_from_camera);
  }

  while (imu_sub_->read(imu_msg, 1)) {
    const jcc::Vec3 accel_mpss(imu_msg.accel_mpss_x, imu_msg.accel_mpss_y, imu_msg.accel_mpss_z);
    const jcc::Vec3 mag_utesla(imu_msg.mag_utesla_x, imu_msg.mag_utesla_y, imu_msg.mag_utesla_z);
    const jcc::Vec3 gyro_radps(imu_msg.gyro_radps_x, imu_msg.gyro_radps_y, imu_msg.gyro_radps_z);

    accel_history_.push_back({accel_mpss, gyro_radps, mag_utesla});
    mag_utesla_.push_back({mag_utesla});
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

  while (fiducial_history_.size() > 10u) {
    fiducial_history_.pop_front();
  }

  while (mag_utesla_.size() > 10u) {
    mag_utesla_.pop_front();
  }

  if (!fiducial_history_.empty()) {
    const SE3 camera_from_fiducial = fiducial_history_.back().inverse();
    transform_network_.update_edge("camera", "fiducial", camera_from_fiducial);

    const jcc::Vec3 jet_origin(1.0, 0.0, 0.0);
    const Pose pose = last_pose_message_.to_pose();
    const SE3 world_from_jet(pose.world_from_jet.so3(), jet_origin);

    sensor_geo_->add_sphere({world_from_jet * camera_from_fiducial.translation(), 0.2, jcc::Vec4(1.0, 1.0, 0.2, 0.8)});
    sensor_geo_->add_axes({world_from_jet * camera_from_fiducial, 0.025, 3.0});
  }

  if (!accel_history_.empty()) {
    sensor_geo_->add_line({jcc::Vec3::Zero(), accel_history_.back().accel_mpss});

    if (transform_network_.edges_from_node_tag().count("world") != 0) {
      const SE3 world_from_imu1 = transform_network_.find_source_from_destination("world", "imu_78");
      sensor_geo_->add_line({world_from_imu1.translation(), world_from_imu1 * accel_history_.back().accel_mpss,
                             jcc::Vec4(0.5, 0.5, 0.8, 1.0)});
    }
  }

  sensor_geo_->add_sphere({jcc::Vec3::Zero(), 9.81});
  sensor_geo_->flip();
}

void FilterVizBq::draw_pose() {
  PoseMessage pose_msg;
  bool got_pose_msg = false;
  while (pose_sub_->read(pose_msg, 1)) {
    last_pose_message_ = pose_msg;
    got_pose_msg = true;
  }

  if (got_pose_msg) {
    const Pose pose = pose_msg.to_pose();

    std::cout << "Pose Age: " << estimation::to_seconds(jcc::now() - to_time_point(pose_msg.timestamp)) << std::endl;

    pose_geo_->add_axes({pose.world_from_jet, 0.55, 3.0, true});

    // const jcc::Vec3 jet_origin(1.0, 0.0, 0.0);
    // const SE3 world_from_jet(pose.world_from_jet.so3(), jet_origin);
    // pose_geo_->add_axes({world_from_jet, 0.55, 3.0, true});

    transform_network_.update_edge("world", "vehicle", pose.world_from_jet);

    geometry::put_transform_network(*pose_geo_, transform_network_, "world");

    pose_geo_->flip();

    const SE3 body_from_model = jcc::exp_z(-M_PI * 0.5);
    // const SE3 body_from_model = jcc::exp_z(0.0);

    jet_tree_->set_world_from_root(pose.world_from_jet * body_from_model);
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
