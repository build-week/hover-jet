#include "geometry/spatial/fit_bounding_box.hh"

#include "eigen.hh"

namespace geometry {
namespace spatial {

using Vec3 = Eigen::Vector3d;

BoundingBox<3> fit_bounding_box(const TriMesh& mesh) {
  BoundingBox<3> bbox;
  for (const auto& tri : mesh.triangles) {
    for (const Vec3& vertex : tri.vertices) {
      bbox.expand(vertex);
    }
  }
  return bbox;
}

}  // namespace spatial
}  // namespace geometry
