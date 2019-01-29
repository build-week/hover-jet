#pragma once

#include <array>
#include <vector>

#include "eigen.hh"

namespace geometry {
struct Triangle {
  std::array<Eigen::Vector3d, 3> vertices;
  Eigen::Vector3d normal;
};

struct TriMesh {
  std::vector<Triangle> triangles;
};
}  // namespace  geometry
