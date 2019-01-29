#pragma once

#include "eigen.hh"
#include "eigen_helpers.hh"
#include "util/fixed_vector.hh"

#include <functional>
#include <vector>

namespace geometry {
namespace spatial {
using Vec3 = Eigen::Vector3d;

struct Simplex {
  static constexpr std::size_t MAX_VERTS = 4;
  jcc::FixedVector<Vec3, MAX_VERTS> vertices;
};

struct Shape {
  std::vector<Simplex> simplices;
};

using VisitorFnc = std::function<void(const Simplex&)>;
void gjk(const Shape& a, const Vec3& b, const VisitorFnc& visitor);

}  // namespace spatial
}  // namespace geometry
