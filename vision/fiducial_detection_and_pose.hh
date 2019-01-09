#pragma once
#include <cassert>
#include <vector>

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {

struct MarkerRvecsTvecs {
  std::vector<cv::Vec<double, 3>> rvecs;
  std::vector<cv::Vec<double, 3>> tvecs;
};

struct MarkerDetection {
  SE3 marker_center_from_camera;
  int id;
};

struct MarkerInWorld {
  SE3 world_from_marker;
  int id;
};

std::vector<MarkerDetection> detect_markers(const cv::Mat& mat);

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat& camera_image,
                                                         const SE3& world_from_camera);

std::optional<SE3> detect_board(const cv::Mat& input_image);

const cv::Ptr<cv::aruco::Dictionary> aruco_dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

constexpr float FIDUCIAL_WIDTH_METERS = 30.25 / 1000;
constexpr float FIDUCIAL_GAP_WIDTH_METERS = 22.98 / 1000;

const cv::Ptr<cv::aruco::GridBoard> aruco_board =
    cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS, FIDUCIAL_GAP_WIDTH_METERS, aruco_dictionary);

}  // namespace jet
