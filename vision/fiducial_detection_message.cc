#include "vision/fiducial_detection_message.hh"

namespace jet {
SE3 FiducialDetectionMessage::fiducial_from_camera() const {
  // Create a copy-less map
  const Eigen::Map<const jcc::Vec6> fiducial_from_camera_log_eigen(fiducial_from_camera_log.data());
  return SE3::exp(fiducial_from_camera_log_eigen);
}

}  // namespace jet