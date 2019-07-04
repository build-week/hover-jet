#include "vision/fiducial_detection_message.hh"

namespace jet {
SE3 FiducialDetectionMessage::fiducial_from_camera() const {
  // Create a copy-less map
  const Eigen::Map<const jcc::Vec6> fiducial_from_camera_log_eigen(fiducial_from_camera_log.data());
  return SE3::exp(fiducial_from_camera_log_eigen);
}

std::optional<FiducialDetectionMessage> create_fiducial_detection_message(const cv::Mat& camera_frame,
                                                              const Calibration& camera_calibration,
                                                              const Timestamp& timestamp) {
  const auto ids_corners = get_ids_and_corners(camera_frame);
  const std::optional<SE3> board_from_camera =
      estimate_board_center_from_camera_from_image(ids_corners, camera_calibration);
  if (board_from_camera) {
    FiducialDetectionMessage detection_message;
    const jcc::Vec6 log_fiducial_from_camera = board_from_camera->log();
    for (int i = 0; i < 6; i++) {
      detection_message.fiducial_from_camera_log[i] = log_fiducial_from_camera[i];
    }
    detection_message.timestamp = timestamp;
    const std::vector<BoardPointImagePointAssociation> board_point_assocs = obj_points_img_points_from_image(ids_corners);
    detection_message.board_points_image_points = board_point_assocs;

    return detection_message;

  }
  return {};
}

}  // namespace jet