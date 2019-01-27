#pragma once

#include <limits>

#include "geometry/tri_mesh.hh"

#include "eigen.hh"

namespace geometry {
namespace spatial {
using Vec3 = Eigen::Vector3d;

struct NearestTriangle {
  double distance = std::numeric_limits<double>::max();
  int simplex_index = -1;
  Vec3 point;
};

NearestTriangle find_nearest_triangle(const Vec3& pt, const TriMesh& shape);
}  // namespace spatial
}  // namespace geometry
