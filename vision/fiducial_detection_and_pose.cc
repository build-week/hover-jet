// }
#include "vision/fiducial_detection_and_pose.hh"
#include <cassert>
#include <cstdlib>

namespace jet {

std::optional<SE3> detect_board(const cv::Mat &input_image) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  const auto params = cv::aruco::DetectorParameters::create();
  params->doCornerRefinement = true;
  params->cornerRefinementWinSize = 5;

  // TODO isaac make aruco_dictionary parameter of this method to allow for 
  // multiple unique boards
  cv::aruco::detectMarkers(input_image, aruco_dictionary, corners, ids, params);

  // TODO isaac move these to config and make them
  const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 499.7749869454186, 0, 309.3792706235992, 0, 496.9300965132637,
       241.6934416030273, 0, 0, 1);
  const cv::Mat distortion_coefficients =
      (cv::Mat1d(1, 5) << 0.003861115403120386, 0.09541932181851349, 0.001898991151152847,
       -0.003082742498836169, -0.2932184860155891);

  // these must be CV mats to force aruco to not use their
  // values as initial pose estimates.  Despair!
  cv::Mat rvec;
  cv::Mat tvec;

  int num_fiducials_detected_on_board = cv::aruco::estimatePoseBoard(
      corners, ids, aruco_board, camera_matrix, distortion_coefficients, rvec, tvec);

  if (tvec.size().height > 0) {
    // The returned transformation is the one that transforms points from each
    // marker coordinate system to the camera coordinate system. The marker
    // corrdinate system is centered on the middle of the marker, with the Z axis
    // perpendicular to the marker plane.
    SE3 camera_from_marker_center = SE3(
        SO3::exp(jcc::Vec3(tvec.at<double>(0, 0), tvec.at<double>(0, 1),
                           tvec.at<double>(0, 2))),
        jcc::Vec3(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2)));
    SE3 marker_center_from_camera = camera_from_marker_center.inverse();

    return {marker_center_from_camera};
  } else {
    return std::nullopt;
  }
}

}  // namespace jet
