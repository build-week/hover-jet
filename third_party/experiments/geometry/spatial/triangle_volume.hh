// Möller–Trumbore
#pragma once

#include "geometry/spatial/volume.hh"

namespace geometry {
namespace spatial {

namespace {
using Vec3 = Eigen::Vector3d;
// Moller-Trumbore
Intersection intersect_triangle(const Vec3 &orig,
                                const Vec3 &dir,
                                const Vec3 &vert0,
                                const Vec3 &vert1,
                                const Vec3 &vert2) {
  // SUB(edge1, vert1, vert0);
  const Vec3 edge1 = vert1 - vert0;
  // SUB(edge2, vert2, vert0);
  const Vec3 edge2 = vert2 - vert0;

  // CROSS(pvec, dir, edge2);
  const Vec3 pvec = dir.cross(edge2);

  // det = DOT(edge1, pvec);
  const double det = edge1.dot(pvec);

  constexpr double EPS = 1e-6;
  if (det > -EPS && det < EPS) {
    return Intersection::no_intersection();
  }
  const double inv_det = 1.0 / det;
  // SUB(tvec, orig, vert0);
  const Vec3 tvec = orig - vert0;

  // *u = DOT(tvec, pvec) * inv_det;
  const double u = tvec.dot(pvec) * inv_det;
  if (u < 0.0 || u > 1.0) {
    return Intersection::no_intersection();
  }
  // CROSS(qvec, tvec, edge1);
  const Vec3 qvec = tvec.cross(edge1);
  // *v = DOT(dir, qvec) * inv_det;
  const double v = dir.dot(qvec) * inv_det;
  if (v < 0.0 || u + v > 1.0) {
    return Intersection::no_intersection();
  }

  // *t = DOT(edge2, qvec) * inv_det;
  const double t = edge2.dot(qvec) * inv_det;
  Intersection intersection;
  intersection.distance = t;
  intersection.intersected = t > 0.0;
  return intersection;
}
}  // namespace

class TriangleVolume final : public Volume {
 public:
  TriangleVolume(const std::array<Vec3, 3> &vertices) : vertices_(vertices) {
  }
  Intersection intersect(const Ray &ray) const override {
    return intersect_triangle(
        ray.origin, ray.direction, vertices_[0], vertices_[1], vertices_[2]);
  }
  Sphere bounding_sphere() const override {
    return {};
  }

  BoundingBox<3> bounding_box() const override {
    BoundingBox<3> box;
    for (const auto &vert : vertices_) {
      box.expand(vert);
    }
    return box;
  }

  bool does_intersect(const Ray &ray) const override {
    return {};
  }

 private:
  std::array<Vec3, 3> vertices_;
};
}  // namespace spatial
}
