#pragma once

#include "sophus.hh"
#include "viewer/primitives/simple_geometry.hh"

#include <functional>
#include <string>

namespace geometry {
namespace visualization {

// Put a mesh into a SimpleGeometry
// geo The SimpleGeometry to insert the mesh into
// world_from_body A transform taking a point in the mesh frame to the world frame
using PutFunction =
    std::function<void(viewer::SimpleGeometry& geo, const SE3& world_from_body)>;

// Create a function that will put a mesh.
PutFunction create_put_stl(const std::string& path_to_stl);

}  // namespace visualization
}  // namespace geometry
