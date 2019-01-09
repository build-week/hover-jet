    // }
#include "vision/fiducial_detection_and_pose.hh"
#include <cassert>
#include <cstdlib>

namespace jet {

namespace {

MarkerRvecsTvecs rvecs_tvecs_from_corners(
    const std::vector<std::vector<cv::Point2f>> &corners) {
  std::vector<cv::Vec3d> rvecs, tvecs;

  // TODO isaac move these to config
  const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 533.086677901258, 0, 318.6124474231535, 0, 526.7508307011435,
       233.3218572898686, 0, 0, 1);
  const cv::Mat distortion_coefficients = (cv::Mat1d(1, 5) << 0.1957195456695698,
                                           -0.738918431892389,
                                           0.004904687104832405,
                                           0.01328395230082144,
                                           0.7438835398485716);
  cv::aruco::estimatePoseSingleMarkers(corners, 0.1335, camera_matrix,
                                       distortion_coefficients, rvecs, tvecs);
  MarkerRvecsTvecs result;
  result.rvecs = rvecs;
  result.tvecs = tvecs;
  return result;
}

}  // namespace

std::vector<MarkerDetection> detect_markers(const cv::Mat &input_image) {
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  const auto params = cv::aruco::DetectorParameters::create();
  params->doCornerRefinement = true;
  cv::aruco::detectMarkers(input_image, dictionary, corners, ids, params);

  MarkerRvecsTvecs rvecs_tvecs = rvecs_tvecs_from_corners(corners);
  if (DRAW_FIDUCIAL_CORNER_DETECTIONS) {
    cv::Mat debug_drawing_image = input_image.clone();
    cv::imshow("window", debug_drawing_image);
    cv::waitKey(1);  // to get window to persist
    for (const auto &quad : corners) {
      for (const auto &center : quad) {
        cv::circle(input_image, center, 10, cv::Scalar(255, 0, 0));
      }
    }
  }

  std::vector<MarkerDetection> detections;
  // draw axis for each marker
  for (int i = 0; i < static_cast<int>(ids.size()); i++) {
    MarkerDetection detection;
    detection.id = ids.at(i);

    const auto camera_from_marker_center =
        SE3(SO3::exp(jcc::Vec3(rvecs_tvecs.rvecs.at(i)[0],
                               rvecs_tvecs.rvecs.at(i)[1],
                               rvecs_tvecs.rvecs.at(i)[2])),
            jcc::Vec3(rvecs_tvecs.tvecs.at(i)[0], rvecs_tvecs.tvecs.at(i)[1],
                      rvecs_tvecs.tvecs.at(i)[2]));
    detection.marker_center_from_camera = camera_from_marker_center.inverse();
    detections.push_back(detection);
  }

  return detections;
}

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat &camera_image,
                                                         const SE3 &world_from_camera) {
  const std::vector<MarkerDetection> marker_detections = detect_markers(camera_image);
  std::vector<MarkerInWorld> result;
  for (auto const &image_detection : marker_detections) {
    const auto marker_center_from_opencv_camera =
        image_detection.marker_center_from_camera;
    const auto opencv_camera_from_marker_center =
        marker_center_from_opencv_camera.inverse();
    const auto world_from_marker_frame =
        (world_from_camera * opencv_camera_from_marker_center);
    const MarkerInWorld world_space_detection{
        .world_from_marker = world_from_marker_frame, .id = image_detection.id};
    result.push_back(world_space_detection);
  }
  return result;
}

std::optional<SE3> detect_board(const cv::Mat &input_image) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  const auto params = cv::aruco::DetectorParameters::create();
  params->doCornerRefinement = true;
  params->cornerRefinementWinSize = 2;

  // TODO make aruco_dictionary parameter
  cv::aruco::detectMarkers(input_image, aruco_dictionary, corners, ids, params);

  // TODO isaac move these to config
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
  int valid = cv::aruco::estimatePoseBoard(corners, ids, aruco_board, camera_matrix,
                                           distortion_coefficients, rvec, tvec);

  if (tvec.size().height > 0) {
    // The returned transformation is the one that transforms points from each
    // marker coordinate system to the camera coordinate system. The marker
    // corrdinate system is centered on the middle of the marker, with the Z axis
    // perpendicular to the marker plane.
    SE3 camera_from_marker_center = SE3(
        SO3::exp(jcc::Vec3(tvec.at<double>(0, 0), tvec.at<double>(0, 1),
                           tvec.at<double>(0, 2))),
        jcc::Vec3(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2)));
    std::cout << camera_from_marker_center.translation().norm();
    SE3 marker_center_from_camera = camera_from_marker_center.inverse();

    if (DRAW_FIDUCIAL_CORNER_DETECTIONS) {
      for (const auto &quad : corners) {
        for (const auto &center : quad) {
          cv::circle(input_image, center, 5, cv::Scalar(255, 0, 0));
        }
      }
    }
    return {marker_center_from_camera};
  } else {
    return std::nullopt;
  }
}

}  // namespace jet
