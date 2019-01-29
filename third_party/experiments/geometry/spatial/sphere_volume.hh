#pragma once

#include "geometry/spatial/volume.hh"

namespace geometry {
namespace spatial {

class SphereVolume final : public Volume {
public:
  SphereVolume(const Eigen::Vector3d &center, const double radius)
      : center_(center)
      , radius_(radius) {}

  Sphere bounding_sphere() const override {
    return {radius_, center_};
  }

  BoundingBox<3> bounding_box() const override {
    BoundingBox<3> box;
    box.expand(radius_ * Eigen::Vector3d::Ones());
    box.expand(-radius_ * Eigen::Vector3d::Ones());
    return box;
  }

  bool does_intersect(const Ray &ray) const override {
    //
    if ((ray.origin - center_).norm() <= radius_) {
      return true;
    } else {
      if (intersect(ray).intersected) {
        return true;
      }
    }
    return false;
  }

  Intersection intersect(const Ray &ray) const override {
    const double r2 = radius_ * radius_;

    const Eigen::Vector3d v = ray.direction;
    const Eigen::Vector3d p = ray.origin;
    const Eigen::Vector3d x0 = center_;

    const double vtv = v.dot(v);
    const double z = p.dot(p) - 2.0 * p.dot(x0) + x0.dot(x0);
    const double b = 2.0 * v.dot(p - x0);
    const double discriminant = (b * b) - 4.0 * vtv * (z - r2);

    Intersection intersection;
    if (discriminant < 0) {
      intersection.intersected = false;
    } else {
      const double t_1 = (-b - std::sqrt(discriminant)) / (2.0 * vtv);
      const double t_2 = (-b + std::sqrt(discriminant)) / (2.0 * vtv);

      if ((t_1 > 0.0) && (t_2 > 0.0)) {
        intersection.distance = std::min(t_1, t_2);
        intersection.intersected = true;
      } else if ((t_1 > 0.0) || (t_2 > 0.0)) {
        intersection.distance = std::max(t_1, t_2);
        intersection.intersected = true;
      } else {
        intersection.intersected = false;
      }
    }
    return intersection;
  }

private:
  Eigen::Vector3d center_;
  double radius_;
};
}
}
