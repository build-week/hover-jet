#include "geometry/visualization/put_stl.hh"

#include "geometry/import/read_stl.hh"

#include "eigen.hh"

namespace geometry {
namespace visualization {

PutFunction create_put_stl(const std::string& path_to_stl) {
  const auto tri = geometry::import::read_stl(path_to_stl);
  assert(tri);
  const auto put = [mesh = *tri](viewer::SimpleGeometry& geo,
                                 const SE3& world_from_body) {
    const jcc::Vec4 color(0.8, 0.8, 0.8, 0.5);
    geo.add_triangle_mesh({mesh, world_from_body, color, true, 3.0, false});
  };
  return put;
}

}  // namespace visualization
}  // namespace geometry
