#include "estimation/vision/fiducial_pose.hh"
#include "eigen.hh"
//%deps(opencv)

namespace estimation {
namespace vision {

std::vector<MarkerDetection> detect_markers(const cv::Mat& inputImage) {
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  // cv::aruco::DetectorParameters params;
  const auto params = cv::aruco::DetectorParameters::create();
  // params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

  const cv::Mat im2 = inputImage.clone();
  cv::aruco::detectMarkers(im2, dictionary, corners, ids, params);
  std::vector<cv::Vec3d> rvecs, tvecs;

  // cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 320, 0, 320, 0, 320, 320, 0, 0, 1);
  // const cv::Mat distortion_coefficients = (cv::Mat1d(1, 8) << 0, 0, 0, 0, 0);

  /*const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 265.9604351267379, 0, 323.3841822012849, 0, 302.7743119963964,
       177.7795703229708, 0, 0, 1);
  const cv::Mat distortion_coefficients = (cv::Mat1d(1, 5) << -0.0881294556831833,
                                           0.08627577358744372,
                                           -0.006574803742454203,
                                           0.01200680448589873,
                                           -0.02887266477084746);
*/
  const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 533.086677901258, 0, 318.6124474231535, 0, 526.7508307011435,
       233.3218572898686, 0, 0, 1);
  const cv::Mat distortion_coefficients = (cv::Mat1d(1, 5) << 0.1957195456695698,
                                           -0.738918431892389,
                                           0.004904687104832405,
                                           0.01328395230082144,
                                           0.7438835398485716);

  constexpr double SIZE_M = 0.14;
  cv::aruco::estimatePoseSingleMarkers(corners, SIZE_M, camera_matrix,
                                       distortion_coefficients, rvecs, tvecs);

  // The returned transformation is the one that transforms points from each marker
  // coordinate system to the camera coordinate system. The marker corrdinate system is
  // centered on the middle of the marker, with the Z axis perpendicular to the marker
  // plane.
  std::vector<MarkerDetection> detections;
  // draw axis for each marker
  for (int i = 0; i < static_cast<int>(ids.size()); i++) {
    const auto camera_from_marker_center =
        SE3(SO3::exp(jcc::Vec3(rvecs[i][0], rvecs[i][1], rvecs[i][2])),
            jcc::Vec3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));

    MarkerDetection detection;
    detection.marker_center_from_camera = camera_from_marker_center.inverse();
    detection.id = ids[i];

    for (const auto& pt : corners[i]) {
      detection.image_points.emplace_back(pt.x, pt.y);
    }

    detections.push_back(detection);
  }

  return detections;
}

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat& camera_image,
                                                         const SE3& world_from_camera) {
  const std::vector<MarkerDetection> marker_detections =
      estimation::vision::detect_markers(camera_image);
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

}  // namespace vision
}  // namespace estimation
