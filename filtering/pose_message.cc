#include "filtering/pose_message.hh"

namespace jet {

Pose PoseMessage::to_pose() const {
  Pose pose;
  const Eigen::Map<const jcc::Vec6> world_from_jet_eigen(log_world_from_jet.data());
  pose.world_from_jet SE3::exp(world_from_jet_eigen);

  pose.v_world_frame = Eigen::Map<const jcc::Vec3>(v_world_frame);
  pose.w_world_frame = Eigen::Map<const jcc::Vec3>(w_world_frame);
  return pose;
}

}  // namespace jet