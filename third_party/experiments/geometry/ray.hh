#pragma once

#include "eigen.hh"

#include "sophus.hh"

namespace geometry {

struct Ray {
  using Vec3 = Eigen::Vector3d;
  Vec3 origin;
  Vec3 direction;

  Vec3 operator()(const double t) const {
    return origin + (t * direction);
  }
};

Ray operator*(const SE3& destination_from_source, const Ray& ray);
}