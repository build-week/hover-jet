#pragma once

#include "eigen.hh"

namespace geometry {
namespace shapes {

struct Plane {
  Eigen::Vector3d u_normal;
  double distance_from_origin;
};

// Signed distance to a half-space
// u_normal must be a unit vector
//
// d = n̂⋅(p - d⋅n̂)
//
// Where d is signed.
inline double sd_halfspace(const Eigen::Vector3d& point, const Plane& plane) {
  // Bam.
  return (point - (plane.distance_from_origin * plane.u_normal)).dot(plane.u_normal);
}

}  // namespace shapes
}  // namespace geometry