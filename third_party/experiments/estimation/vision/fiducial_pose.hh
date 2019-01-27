#pragma once
#include <cassert>
#include <vector>

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace vision {

struct MarkerDetection {
  SE3 marker_center_from_camera;
  std::vector<jcc::Vec2> image_points;
  int id;
};

struct MarkerInWorld {
  SE3 world_from_marker;
  int id;
};

std::vector<MarkerDetection> detect_markers(const cv::Mat& mat);

std::vector<MarkerInWorld> get_world_from_marker_centers(const cv::Mat& camera_image,
                                                         const SE3& world_from_opengl_camera);

}  // namespace vision
}  // namespace estimation