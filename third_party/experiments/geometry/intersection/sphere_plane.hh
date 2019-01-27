#pragma once

#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"

#include "util/optional.hh"

#include "eigen.hh"

#include <cmath>

namespace geometry {
namespace intersection {

jcc::Optional<shapes::Circle> sphere_plane_intersection(const shapes::Sphere& sphere,
                                                        const shapes::Plane& plane) {
  const double signed_chord_height = sd_halfspace(sphere.center, plane);
  if (std::abs(signed_chord_height) > sphere.radius) {
    return {};
  }

  // Call upon Pythagoras!
  const double radius_sq = sphere.radius * sphere.radius;
  const double chord_height_sq = signed_chord_height * signed_chord_height;
  const double r = std::sqrt(radius_sq - chord_height_sq);

  const Eigen::Vector3d circle_center = (-signed_chord_height * plane.u_normal) + sphere.center;
  const shapes::Circle circle{circle_center, plane.u_normal, r};
  return circle;
}

}  // namespace intersection
}  // namespace geometry