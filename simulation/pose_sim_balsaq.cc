//%bin(pose_sim_balsaq_main)
#include "simulation/pose_sim_balsaq.hh"

#include "embedded/imu_driver/imu_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include "filtering/pose_message.hh"

#include <iostream>

#include "third_party/experiments/sophus.hh"

namespace jet {
namespace simulation {

PoseSimulatorBq::PoseSimulatorBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void PoseSimulatorBq::init(const Config& config) {
  std::cout << "Subscribing IMU" << std::endl;
  imu_sub_ = make_subscriber("imu");

  std::cout << "Advertising Pose" << std::endl;
  pose_pub_ = make_publisher("pose");
}

void PoseSimulatorBq::loop() {
  ImuMessage imu_msg;
  bool got_imu_msg = false;

  while (imu_sub_->read(imu_msg, 1)) {
    got_imu_msg = true;
  }

  Pose pose;
  // TODO: Create simulated time
  if (got_imu_msg) {
    const SE3 T_body_from_world;

    pose.world_from_jet.translation() = jcc::Vec3::Zero();
    pose.world_from_jet.so3() = SO3::exp(jcc::Vec3(0.0, 0.0, -1.5));

    pose.v_world_frame = jcc::Vec3(0.0, 0.0, 0.0);
    pose.w_world_frame = jcc::Vec3(0.0, 0.0, 0.0);
    pose.timestamp = imu_msg.timestamp;

    PoseMessage pose_msg = PoseMessage::from_pose(pose);
    std::cout << "Transmitting: " << pose.world_from_jet.log().transpose() << std::endl;
    pose_pub_->publish(pose_msg);
  }
}

void PoseSimulatorBq::shutdown() {
  std::cout << "Pose Simulator shutdown" << std::endl;
}

}  // namespace simulation
}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::simulation::PoseSimulatorBq)
