#pragma once

#include "viewer/primitives/simple_geometry.hh"

#include "geometry/import/read_collada.hh"

namespace geometry {
namespace visualization {

void put_collada(viewer::SimpleGeometry& geo,
                 const geometry::import::ColladaModel& model,
                 const SE3 world_from_root);

}  // namespace visualization
}  // namespace geometry