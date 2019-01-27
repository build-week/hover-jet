#include "geometry/spatial/octree.hh"

#include "geometry/spatial/bounding_box.hh"

namespace geometry {
namespace spatial {

// Build is not designed to be very fast
void Octree::build(const std::vector<Vec3> &points) {
  BoundingBox<3> bbox;
  for (const auto &pt : points) {
    bbox.expand(pt);
  }

  // nodes_
  // points_
}
}
}
