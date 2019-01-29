#pragma once

#include "eigen.hh"

namespace geometry {
namespace shapes {
struct Sphere {
  const Eigen::Vector3d center;
  const double radius;
};

// Signed distance to a sphere
inline double sd_sphere(const Eigen::Vector3d& point, const Sphere& sphere) {
  return (point - sphere.center).norm() - sphere.radius;
}

}  // namespace shapes
}  // namespace geometry