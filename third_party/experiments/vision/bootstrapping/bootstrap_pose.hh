#pragma once

#include <vector>

#include "eigen.hh"

#include "out.hh"
#include "sophus.hh"

#include "vision/camera_model.hh"

namespace slam {

struct BootstrapResult {
  using Vec3 = Eigen::Vector3d;
  bool success;

  double rms_residual = 0.0;
  double rcond = 0.0;
  // Pose with unit translation
  SE3                 view_b_from_view_a_step;
  std::vector<double> weights;
  std::vector<Vec3>   estimated_structure;
};

// image_points_a and image_points_b must be of the same size
BootstrapResult compute_nonmetric_pose(const std::vector<Eigen::Vector2d>& image_points_a,
                                       const std::vector<Eigen::Vector2d>& image_points_b,
                                       const CameraModel&                  camera_model,
                                       const SE3&                          initial_view_b_from_a);
}