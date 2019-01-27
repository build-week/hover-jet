#include "geometry/spatial/nearest_triangle.hh"

#include "geometry/shapes/triangle.hh"

namespace geometry {
namespace spatial {

NearestTriangle find_nearest_triangle(const Vec3& pt, const TriMesh& shape) {
  NearestTriangle nearest;
  int i = 0;
  for (const auto& triangle : shape.triangles) {
    const Vec3 closest_pt = shapes::find_closest_point_on_triangle(
        pt, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);

    const double sq_dist = (pt - closest_pt).squaredNorm();
    if (sq_dist < nearest.distance) {
      nearest.distance = sq_dist;
      nearest.point = closest_pt;
      nearest.simplex_index = i;
    }
    ++i;
  }
  return nearest;
}

}  // namespace spatial
}  // namespace geometry
