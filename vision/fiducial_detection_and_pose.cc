#include "vision/fiducial_detection_and_pose.hh"
#include <cassert>
#include <cstdlib>

namespace jet { 

std::tuple<std::vector<int> , std::vector<std::vector<cv::Point2f>>> get_ids_and_corners(const cv::Mat &input_image){
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  const auto params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX;
  params->cornerRefinementWinSize = 5;

  // TODO isaac make aruco_dictionary parameter of this method to allow for
  // multiple unique boards
  cv::aruco::detectMarkers(input_image, get_aruco_dictionary(), corners, ids, params);
  return std::make_tuple(ids, corners);
}

std::vector<BoardPointImagePointAssociation> obj_points_img_points_from_image(const cv::Mat &input_image){
  const auto ids_corners = get_ids_and_corners(input_image);
  const auto ids = std::get<0>(ids_corners);
  const auto corners = std::get<1>(ids_corners);
  cv::Mat boardPoints, imgPoints;
  cv::aruco::getBoardObjectAndImagePoints(get_aruco_board(), corners, ids, boardPoints, imgPoints);
  std::vector<BoardPointImagePointAssociation> result;
  for (int i=0; i < boardPoints.rows; i++){
    BoardPointImagePointAssociation association = {};
    boardPoints.at<float>(i, 0);
    association.point_board_space = jcc::Vec2(boardPoints.at<float>(i, 0), boardPoints.at<float>(i, 1));
    association.point_image_space = jcc::Vec2(imgPoints.at<float>(i, 0), imgPoints.at<float>(i, 1));
    result.push_back(association);
  }
  return result;
}

std::optional<SE3> estimate_board_center_from_camera_from_image(const cv::Mat &input_image) {
  
  const auto ids_corners = get_ids_and_corners(input_image);
  const auto ids = std::get<0>(ids_corners);
  const auto corners = std::get<1>(ids_corners);

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

  const int num_fiducials_detected_on_board =
      cv::aruco::estimatePoseBoard(corners, ids, get_aruco_board(), camera_matrix,
                                   distortion_coefficients, rvec, tvec);
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
