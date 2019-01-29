#include "geometry/shapes/sdf_box.hh"

#include "geometry/import/read_stl.hh"
#include "sophus.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/shapes/triangle.hh"

namespace geometry {
namespace shapes {

namespace {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

void add_triangle(viewer::SimpleGeometry& geom,
                  const Vec3& a,
                  const Vec3& b,
                  const Vec3& c,
                  const Vec4& color,
                  double width = 1.0) {
  geom.add_line({a, b, color, width});
  geom.add_line({b, c, color, width});
  geom.add_line({c, a, color, width});
}
}  // namespace

TriMesh add_cube() {
  auto win = viewer::get_window3d("Voxel Box");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();

  const std::string file_path =
      "/home/jacob/repos/experiments/data/sphere_cube_shape.stl";
  const auto tri = geometry::import::read_stl(file_path);
  assert(tri);

  const Vec4 color(1.0, 1.0, 1.0, 0.6);

  for (std::size_t k = 0; k < tri->triangles.size(); ++k) {
    add_triangle(*geom,
                 tri->triangles[k].vertices[0],
                 tri->triangles[k].vertices[1],
                 tri->triangles[k].vertices[2],
                 color);
  }
  geom->flip();
  return *tri;
}

void demo_sdf() {
  const auto shape = add_cube();

  auto win = viewer::get_window3d("Voxel Box");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();
  auto beauty = win->add_primitive<viewer::SimpleGeometry>();

  viewer::Points points;
  std::vector<double> intensities;
  points.size = 5.0;

  const SampledSdf sdf(shape);

  geom->add_box({sdf.bounding_box().lower(), sdf.bounding_box().upper()});

  const auto& dists = sdf.get_signed_distances();
  for (std::size_t k = 0u; k < dists.size(); ++k) {
    points.points.push_back(sdf.position_for_voxel_index(k));
    intensities.push_back(dists[k]);
  }

  geom->add_colored_points(points, intensities);
  geom->flip();

  const Vec3 base_pt = Vec3(3.0, 3.0, 3.0);
  for (double r = 0.0; r < 90.14; r += 0.05) {
    const SO3 rot = SO3::exp(Vec3(r, -r, std::exp(std::cos(r))));
    const Vec3 p = rot * base_pt;

    const Vec3 nearest = sdf.bounding_box().nearest_point(p);
    beauty->add_line({p, nearest});
    beauty->add_point({p, Vec4(1.0, 0.0, 0.0, 0.8), 10.0});

    {
      const Vec3 inward = (sdf.bounding_box().center() - nearest).normalized();
      const Vec3 closer = nearest + (inward * 0.05);

      beauty->add_point({closer, Vec4(0.7, 1.0, 0.0, 0.8), 10.0});

      const int i = sdf.voxel_index_for_position(closer);
      std::cout << i << std::endl;
      std::cout << closer.transpose() << " : ";
      const Vec3 voxel = sdf.position_for_voxel_index(i);
      beauty->add_point({voxel, Vec4(0.3, 0.5, 0.8, 0.8), 10.0});
      std::cout << voxel.transpose() << std::endl;
    }

    beauty->flip();
    win->spin_until_step();
  }

  win->spin_until_step();
}

}  // namespace shapes
}  // namespace geometry

int main() {
  geometry::shapes::demo_sdf();
  return 0;
}