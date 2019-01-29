#pragma once

#include "eigen.hh"

#include "geometry/ray.hh"

namespace geometry {
struct IntersectionParameters {
  double along_a;
  double along_b;
  bool   valid = false;
};

// Approximately intersect two rays
//
IntersectionParameters ray_ray_approx_intersect(const Ray& a, const Ray& b);
}