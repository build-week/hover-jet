#include "vision/fiducial_detection_and_pose.hh"
#include <cassert>
#include <cstdlib>

namespace jet {

namespace {

MarkerRvecsTvecs rvecs_tvecs_from_corners(
    const std::vector<std::vector<cv::Point2f>>& corners) {
  std::vector<cv::Vec3d> rvecs, tvecs;
  const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 320, 0, 320, 0, 320, 320, 0, 0, 1);
  const cv::Mat distortion_coefficients =
      (cv::Mat1d(1, 8) << 0, 0, 0, 0, 0, 0, 0, 0);
  cv::aruco::estimatePoseSingleMarkers(corners, 0.49375, camera_matrix,
                                       distortion_coefficients, rvecs, tvecs);
  MarkerRvecsTvecs result;
  result.rvecs = rvecs;
  result.tvecs = tvecs;
  return result;
}

}  // namespace

std::vector<MarkerDetection> detect_markers(cv::Mat& input_image) {
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(input_image, dictionary, corners, ids);
  MarkerRvecsTvecs rvecs_tvecs = rvecs_tvecs_from_corners(corners);

  if (DRAW_FIDUCIAL_CORNER_DETECTIONS) {
    for (const auto& quad : corners) {
      for (const auto& center : quad) {
        cv::circle(input_image, center, 10, cv::Scalar(255, 0, 0));
      }
    }
  }
  // The returned transformation is the one that transforms points from each
  // marker coordinate system to the camera coordinate system. The marker
  // corrdinate system is centered on the middle of the marker, with the Z axis
  // perpendicular to the marker plane.
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

std::vector<MarkerInWorld> get_world_from_marker_centers(
    const cv::Mat& camera_image, const SE3& world_from_camera) {
  const std::vector<MarkerDetection> marker_detections =
      detect_markers(camera_image);
  std::vector<MarkerInWorld> result;
  for (auto const& image_detection : marker_detections) {
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

}  // namespace jet
