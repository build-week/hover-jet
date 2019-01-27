#pragma once

#include "geometry/ray.hh"
#include "bounding_box.hh"

#include "intersection.hh"

namespace geometry {
namespace spatial {

struct Sphere {
  double radius;
  Eigen::Vector3d center;
};

class Volume {
public:
  virtual Intersection intersect(const Ray &ray) const = 0;

  virtual bool does_intersect(const Ray &ray) const {
    if (intersect(ray).intersected) {
      return true;
    } else {
      return false;
    }
  }

  virtual Sphere bounding_sphere() const {
    const auto bbox = bounding_box();
    const double radius = (bbox.upper() - bbox.lower()).norm() * 0.5;
    const Eigen::Vector3d center = (bbox.upper() + bbox.lower()) * 0.5;
    Sphere sphere;
    sphere.center = center;
    sphere.radius = radius;
    return sphere;
  }
  virtual BoundingBox<3> bounding_box() const = 0;
};
}
}
