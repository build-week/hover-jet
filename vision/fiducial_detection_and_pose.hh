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

std::vector<MarkerInWorld> get_world_from_marker_centers(
    const cv::Mat& camera_image, const SE3& world_from_camera);

}  // namespace jet
