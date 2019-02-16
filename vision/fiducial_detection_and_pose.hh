#pragma once
#include <cassert>
#include <vector>

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {

struct MarkerDetection {
  SE3 marker_center_from_camera;
  int id;
};

struct MarkerInWorld {
  SE3 world_from_marker;
  int id;
};

std::vector<MarkerDetection> detect_markers(const cv::Mat& mat);

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat& camera_image, const SE3& world_from_camera);

std::optional<SE3> estimate_board_center_from_camera_from_image(const cv::Mat& input_image);

std::tuple<cv::Mat, cv::Mat> obj_points_img_points_from_image(const cv::Mat &input_image);

constexpr float FIDUCIAL_WIDTH_METERS = 99.0 / 1000;
constexpr float FIDUCIAL_GAP_WIDTH_METERS = 50.0 / 1000;

inline cv::Ptr<cv::aruco::Dictionary> get_aruco_dictionary() {
  return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
}

inline cv::Ptr<cv::aruco::GridBoard> get_aruco_board() {
  return cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS, FIDUCIAL_GAP_WIDTH_METERS, get_aruco_dictionary());
}

}  // namespace jet
