#pragma once

#include "geometry/spatial/bounding_box.hh"
#include "geometry/tri_mesh.hh"

namespace geometry {
namespace spatial {

BoundingBox<3> fit_bounding_box(const TriMesh& mesh);

}  // namespace spatial
}  // namespace geometry
