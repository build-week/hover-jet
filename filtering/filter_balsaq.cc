//%bin(filter_balsaq_main)
#include "filtering/filter_balsaq.hh"

#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "filtering/pose_message.hh"
#include "vision/fiducial_detection_message.hh"

#include "filtering/convert_messages.hh"
#include "filtering/imu_model_from_yaml.hh"
#include "filtering/to_time_point.hh"
#include "filtering/transform_network_from_yaml.hh"

#include <cstddef>
#include <iostream>

namespace jet {
namespace filtering {

constexpr int IMU_1_ID = 78;
constexpr int IMU_2_ID = 36;

FilterBq::FilterBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void FilterBq::init(const Config& config) {
  gonogo().nogo("Starting up");
  std::cout << "Starting up" << std::endl;

  std::cout << "Subscribing Fiducial..." << std::endl;
  fiducial_sub_ = make_subscriber("fiducial_detection_channel");
  std::cout << "Advertising Pose..." << std::endl;
  pose_pub_ = make_publisher("pose");

  //
  // Load intrinsics for each IMU
  //

  ejf::FilterManagerConfiguration filter_cfg;

  std::cout << "Setting up transform network" << std::endl;
  filter_cfg.max_fiducial_latency_s = config["max_fiducial_latency_s"].as<double>();
  filter_cfg.transform_network = transform_network_from_yaml(config["transforms"]);

  std::cout << "Loading IMU calibrations..." << std::endl;
  {
    const auto cfg_imu_1 = imu_model_from_yaml(config["imus"][IMU_1_ID]);
    const auto cfg_imu_2 = imu_model_from_yaml(config["imus"][IMU_2_ID]);

    filter_cfg.imu_model_from_id[cfg_imu_1.imu_id] = cfg_imu_1.imu_model;
    filter_cfg.imu_model_from_id[cfg_imu_2.imu_id] = cfg_imu_2.imu_model;

    std::cout << "Starting imu subscriptions..." << std::endl;
    imu_sub_from_id_[cfg_imu_1.imu_id] = make_subscriber("imu_1");
    imu_sub_from_id_[cfg_imu_2.imu_id] = make_subscriber("imu_2");
  }

  filter_manager_.init(filter_cfg);
}

void FilterBq::loop() {
  //
  // Update with IMU measurements
  //

  ImuMessage imu_msg;
  while (imu_sub_from_id_[IMU_1_ID]->read(imu_msg, 1)) {
    filter_manager_.measure_gyro(to_gyro_meas(imu_msg));
    filter_manager_.measure_imu(to_accel_meas(imu_msg), IMU_1_ID);
  }

  FiducialDetectionMessage detection_msg;
  while (fiducial_sub_->read(detection_msg, 1)) {
    filter_manager_.measure_fiducial(to_fiducial_meas(detection_msg));
  }

  const auto current_time = to_time_point(get_current_time());
  filter_manager_.update(current_time);

  //
  // Report a state
  //
  if (filter_manager_.stage() == ejf::FilterStage::RUNNING) {
    gonogo().go("Running");
    const auto state = filter_manager_.state(current_time);

    Pose pose;
    {
      const SE3 T_world_from_body = estimation::jet_filter::get_world_from_body(state.x);
      pose.world_from_jet = T_world_from_body;

      pose.v_world_frame = state.x.eps_dot.head<3>();
      pose.w_world_frame = state.x.eps_dot.tail<3>();
      pose.timestamp = Timestamp(state.time_of_validity);
    }

    // Publish is non-const
    PoseMessage pose_msg = PoseMessage::from_pose(pose);
    pose_pub_->publish(pose_msg);
  } else {
    gonogo().nogo(ejf::stage_to_string(filter_manager_.stage()));
  }
}  // namespace filtering

void FilterBq::shutdown() {
  std::cout << "Filter shutdown" << std::endl;
}

}  // namespace filtering
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::filtering::FilterBq)
