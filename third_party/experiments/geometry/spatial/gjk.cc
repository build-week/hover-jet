#include "geometry/spatial/gjk.hh"

#include "geometry/perp.hh"

// Rendering
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace spatial {

namespace {
Vec3 support(const Shape& shape, const Vec3& direction) {
  const Vec3* best_vec = nullptr;
  double max_dot = -1.0;

  for (std::size_t k = 0u; k < shape.simplices.size(); ++k) {
    constexpr std::size_t BULLSHIT_CONSTANT = 4u;
    for (std::size_t j = 0u; j < BULLSHIT_CONSTANT; ++j) {
      const Vec3& vertex = shape.simplices[k].vertices[j];
      const double dot = vertex.dot(direction);
      if (dot > max_dot) {
        max_dot = dot;
        best_vec = &vertex;
      }
    }
  }

  return *best_vec;
}

struct NearestSimplexResult {
  Simplex simplex;
  Vec3 toward_origin;
};

// Simplex nearest_simplex(const Simplex& simplex) {
//   //
// }

NearestSimplexResult gjk_nearest_simplex(const Simplex& simplex) {
  NearestSimplexResult result;
  result.simplex = simplex;
  switch (simplex.vertices.size()) {
    case 0:  // Nonsense
      assert(false);
    case 1: {  // Point
      result.toward_origin = -simplex.vertices[0];
      break;
    }
    case 2: {  // Line
      const Vec3 dir = simplex.vertices[1] - simplex.vertices[0];
      const Vec3 coplane_normal = simplex.vertices[0].cross(dir);
      const Vec3 axis = coplane_normal.cross(dir).normalized();
      if (axis.dot(simplex.vertices[0]) < 0.0) {
        result.toward_origin = -axis;
      }
      result.toward_origin = axis;
      break;
    }
    case 3: {  // Triangle
      const Vec3 axis = simplex.vertices[1].cross(simplex.vertices[0]);
      if (axis.dot(simplex.vertices[0]) < 0.0) {
        result.toward_origin = -axis;
      }
      result.toward_origin = axis;
      break;
    }

    case 4: {
      std::cout << "Not ready" << std::endl;
      break;
    }
  };
  return result;
}
}  // namespace

void gjk(const Shape& a, const Vec3& b, const VisitorFnc& visitor) {
  Vec3 axis = Vec3::UnitX();
  Simplex collision_simplex;

  for (int k = 0; k < 4; ++k) {
    Vec3 support_point = support(a, axis) - b;
    collision_simplex.vertices.push_back(support_point);

    const auto gjk_ns_result = gjk_nearest_simplex(collision_simplex);
    axis = gjk_ns_result.toward_origin;
    collision_simplex = gjk_ns_result.simplex;
    visitor(collision_simplex);
  }
}

}  // namespace spatial
}  // namespace geometry
