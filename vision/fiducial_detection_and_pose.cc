#include "vision/fiducial_detection_and_pose.hh"
#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "infrastructure/time/duration.hh"
#include "infrastructure/time/time_utils.hh"

#include <cassert>
#include <cstdlib>

namespace jet {

BoardIdsAndCorners get_ids_and_corners(const cv::Mat& input_image) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  const auto params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG;
  // CORNER_REFINE_NONE, no refinement.
  // CORNER_REFINE_SUBPIX, do subpixel refinement.
  // CORNER_REFINE_CONTOUR use contour-Points
  // CORNER_REFINE_APRILTAG use the AprilTag2 approach)
  params->cornerRefinementWinSize = 5;
  params->cornerRefinementMinAccuracy = .01;  // deafult .1
  params->adaptiveThreshWinSizeMax = 8;       // default 23
  params->aprilTagQuadSigma = 0;              // default undocumented

  // TODO isaac make aruco_dictionary parameter of this method to allow for
  // multiple unique boards
  const auto t0 = time::get_current_time();
  cv::aruco::detectMarkers(input_image, get_aruco_dictionary(), corners, ids, params);
  std::cout << "to do cv::aruco::detectMarkers " << (float)(time::get_current_time() - t0) / 1000000 << "ms"
            << std::endl;
  BoardIdsAndCorners result = {ids, corners};
  return result;
}

std::vector<BoardPointImagePointAssociation> obj_points_img_points_from_image(const BoardIdsAndCorners& ids_corners) {
  cv::Mat boardPoints, imgPoints;
  cv::aruco::getBoardObjectAndImagePoints(get_aruco_board(), ids_corners.corners, ids_corners.ids, boardPoints,
                                          imgPoints);
  std::vector<BoardPointImagePointAssociation> result;

  for (int i = 0; i < board_points.rows; i++) {
    BoardPointImagePointAssociation association = {};
    association.point_board_space = jcc::Vec2(board_points.at<float>(i, 0), board_points.at<float>(i, 1));
    association.point_image_space = jcc::Vec2(img_points.at<float>(i, 0), img_points.at<float>(i, 1));
    result.push_back(association);
  }
  return result;
}

std::optional<SE3> estimate_board_bottom_left_from_camera(const BoardIdsAndCorners& ids_corners,
                                                          const Calibration& calibration) {
  const cv::Mat camera_matrix = calibration.camera_matrix;
  const cv::Mat distortion_coefficients = calibration.distortion_coefficients;

  // these must be CV mats to force aruco to not use their
  // values as initial pose estimates.  Despair!
  cv::Mat rvec;
  cv::Mat tvec;

  const int num_fiducials_detected_on_board = cv::aruco::estimatePoseBoard(
      ids_corners.corners, ids_corners.ids, get_aruco_board(), camera_matrix, distortion_coefficients, rvec, tvec);
  (void)num_fiducials_detected_on_board;

  if (tvec.size().height > 0) {
    // The returned transformation is the one that transforms points from each
    // marker coordinate system to the camera coordinate system. The marker
    // corrdinate system is centered on the middle of the marker, with the Z axis
    // perpendicular to the marker plane.
    const SE3 camera_from_marker_center =
        SE3(SO3::exp(jcc::Vec3(rvec.at<double>(0, 0), rvec.at<double>(0, 1), rvec.at<double>(0, 2))),
            jcc::Vec3(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2)));
    const SE3 marker_center_from_camera = camera_from_marker_center.inverse();

    return {marker_center_from_camera};
  } else {
    return std::nullopt;
  }
}

}  // namespace jet
