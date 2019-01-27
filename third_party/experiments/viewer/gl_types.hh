#pragma once

#include "eigen.hh"

namespace viewer {

struct WindowPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WindowPoint() = default;
  WindowPoint(const Eigen::Vector2d& point_) : point(point_){};
  Eigen::Vector2d                    point;
};

struct ViewportPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ViewportPoint() = default;
  ViewportPoint(const Eigen::Vector2d& point_) : point(point_){};
  Eigen::Vector2d                      point;
};

}  // viewer
