#pragma once

#include "eigen.hh"

#include "geometry/shapes/halfspace.hh"

namespace geometry {
namespace shapes {

struct Circle {
  Eigen::Vector3d center;
  Eigen::Vector3d u_normal;
  double radius;
};

}  // namespace shapes
}  // namespace geometry
