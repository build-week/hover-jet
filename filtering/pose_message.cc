#include "filtering/pose_message.hh"

namespace jet {

Pose PoseMessage::to_pose() const {
  Pose pose;
  const Eigen::Map<const jcc::Vec6> world_from_jet_eigen(log_world_from_jet.data());
  pose.world_from_jet = SE3::exp(world_from_jet_eigen);

  pose.v_world_frame = Eigen::Map<const jcc::Vec3>(v_world_frame.data());
  pose.w_world_frame = Eigen::Map<const jcc::Vec3>(w_world_frame.data());
  return pose;
}

PoseMessage PoseMessage::from_pose(const Pose& pose) {
  PoseMessage msg;

  const jcc::Vec6 log_world_from_jet = pose.world_from_jet.log();
  msg.log_world_from_jet[0] = log_world_from_jet[0];
  msg.log_world_from_jet[1] = log_world_from_jet[1];
  msg.log_world_from_jet[2] = log_world_from_jet[2];
  msg.log_world_from_jet[3] = log_world_from_jet[3];
  msg.log_world_from_jet[4] = log_world_from_jet[4];
  msg.log_world_from_jet[5] = log_world_from_jet[5];

  msg.v_world_frame[0] = pose.v_world_frame[0];
  msg.v_world_frame[1] = pose.v_world_frame[1];
  msg.v_world_frame[2] = pose.v_world_frame[2];

  msg.w_world_frame[0] = pose.w_world_frame[0];
  msg.w_world_frame[1] = pose.w_world_frame[1];
  msg.w_world_frame[2] = pose.w_world_frame[2];

  msg.timestamp = pose.timestamp;
  return msg;
}

}  // namespace jet