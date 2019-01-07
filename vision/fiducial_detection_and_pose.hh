#pragma once
#include <cassert>
#include <vector>

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "third_party/experiments/eigen.hh"
#include "third_party/experiments/sophus.hh"

namespace jet {

constexpr bool DRAW_FIDUCIAL_CORNER_DETECTIONS = true;

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

void detect_board(const cv::Mat& input_image);

const cv::Ptr<cv::aruco::Dictionary> aruco_dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
const cv::Ptr<cv::aruco::GridBoard> aruco_board =
    cv::aruco::GridBoard::create(4, 4, 35.15 / 1000, 35.15 / 1000, aruco_dictionary);

}  // namespace jet
