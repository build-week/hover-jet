#pragma once

#include "camera_model.hh"
#include "robust_estimator.hh"

#include "out.hh"
#include "sophus/se3.hpp"

#include "eigen.hh"
#include <vector>

namespace slam {

struct Correspondence {
  int origin_index;
  int destination_index;
  int hm_distance;
};

struct ImageAlignmentConfig {};

struct ImageAlignmentContext {};

struct ImageAlignmentResult {
  using Scalar = double;
  using SE3    = Sophus::SE3<Scalar>;
  using Mat6   = Eigen::Matrix<double, 6, 6>;

  bool                success      = false;
  double              rms_residual = 0.0;
  double              rcond        = 1.0;
  std::vector<double> weights;
  Mat6                info_at_convergence = Mat6::Zero();
  // Camera from object
  SE3 delta;
};

class ImageAligner {
 public:
  using Scalar = double;
  using Vec2   = Eigen::Vector2d;
  using Vec3   = Eigen::Vector3d;
  using Vec6   = Eigen::Matrix<Scalar, 6, 1>;
  using SE3    = Sophus::SE3<Scalar>;

  ImageAligner(const ImageAlignmentConfig& config = {}) : config_(config){};

  // Solve pnp
  // todo: cool kids solve p3p in closed form
  ImageAlignmentResult perspective_npoint(const CameraModel&         cam_model,
                                          const SE3&                 initial_camera_from_object,
                                          const std::vector<Vec2>&   observed,
                                          const std::vector<Vec3>&   object,
                                          const RobustEstimator&     estimator,
                                          Out<ImageAlignmentContext> ctx) const;

  ImageAlignmentResult standard_align(const CameraModel&       cam_model,
                                      const SE3&               initial_camera_from_object,
                                      const std::vector<Vec2>& observed,
                                      const std::vector<Vec3>& object) const;

 private:
  ImageAlignmentConfig config_;
};
}