#pragma once

#include "eigen.hh"
#include "out.hh"

#include "geometry/line.hh"
#include "geometry/ray.hh"

namespace geometry {

struct Plane {
  using Vec3 = Eigen::Vector3d;

  Vec3 origin;
  Vec3 normal;

  bool intersect(const Line& line, Out<Vec3> point) const;
  bool intersect(const Ray& ray, Out<Vec3> point) const;
};
}  // namespace geometry
