#include "geometry/spatial/gjk.hh"

#include "geometry/import/read_stl.hh"
#include "sophus.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/shapes/triangle.hh"

namespace geometry {
namespace spatial {

namespace {
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

void add_simplex(viewer::SimpleGeometry& geom,
                 const Simplex& s,
                 const Vec4& color,
                 double width = 1.0) {
  if (s.vertices.size() == 1u) {
    geom.add_point({s.vertices[0], color, width});
  } else if (s.vertices.size() == 2u) {
    geom.add_line({s.vertices[0], s.vertices[1], color, width});
  } else if (s.vertices.size() >= 3u) {
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[2], color, width);
  }
  if (s.vertices.size() == 4u) {
    add_triangle(geom, s.vertices[1], s.vertices[2], s.vertices[3], color, width);
    add_triangle(geom, s.vertices[2], s.vertices[0], s.vertices[3], color, width);
    add_triangle(geom, s.vertices[0], s.vertices[1], s.vertices[3], color, width);
  }
}
}  // namespace

Shape add_cube() {
  auto win = viewer::get_window3d("GJK View");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();

  const std::string file_path =
      "/home/jacob/repos/experiments/data/sphere_cube_shape.stl";
  const auto tri = geometry::import::read_stl(file_path);
  assert(tri);

  const Vec4 color(1.0, 1.0, 1.0, 0.6);

  // const SO3 world_from_tri_rot;
  // const SE3 world_from_tri(world_from_tri_rot, Vec3(2.0, 2.0, 2.0));
  const SE3 world_from_tri;

  Shape shape;
  for (size_t k = 0; k < tri->triangles.size(); ++k) {
    Simplex simplex;
    simplex.vertices.push_back(world_from_tri * tri->triangles[k].vertices[0]);
    simplex.vertices.push_back(world_from_tri * tri->triangles[k].vertices[1]);
    simplex.vertices.push_back(world_from_tri * tri->triangles[k].vertices[2]);
    shape.simplices.push_back(simplex);

    add_simplex(*geom, simplex, color);
  }
  geom->flip();
  return shape;
}

Shape make_tetrahedron() {
  Shape shape_1;

  auto win = viewer::get_window3d("GJK View");

  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

  const Vec3 v0 = Vec3::UnitX();
  const Vec3 v1 = Vec3::UnitY();
  const Vec3 v2 = Vec3::UnitZ();
  const Vec3 v3 = -(v0 + v1 + v2) / 3.0;

  Simplex s1;
  s1.vertices.push_back(v0);
  s1.vertices.push_back(v1);
  s1.vertices.push_back(v2);
  s1.vertices.push_back(v3);
  shape_1.simplices.push_back(s1);

  //
  // Draw it all
  //
  {
    for (const auto& simplex : shape_1.simplices) {
      add_simplex(*mesh_a, simplex, Vec4(1.0, 1.0, 1.0, 0.7));
    }
  }
  mesh_a->flip();
  return shape_1;
}

Shape make_triangle() {
  Shape shape_1;

  auto win = viewer::get_window3d("GJK View");

  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

  const Vec3 v0 = Vec3::UnitX();
  const Vec3 v1 = Vec3::UnitY();
  const Vec3 v2 = Vec3::UnitZ();

  Simplex s1;
  s1.vertices.push_back(v0);
  s1.vertices.push_back(v1);
  s1.vertices.push_back(v2);
  shape_1.simplices.push_back(s1);

  //
  // Draw it all
  //
  {
    for (const auto& simplex : shape_1.simplices) {
      add_simplex(*mesh_a, simplex, Vec4(1.0, 1.0, 1.0, 0.7));
    }
  }
  mesh_a->flip();
  return shape_1;
}

struct NearestPoint {
  double distance = std::numeric_limits<double>::max();
  int simplex_index = -1;
  Vec3 point;
};

NearestPoint find_closest_point(const Vec3& pt, const Shape& shape) {
  NearestPoint nearest;
  int i = 0;
  for (const auto& simplex : shape.simplices) {
    const Vec3 closest_pt = shapes::find_closest_point_on_triangle(
        pt, simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]);

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

void demo_intersection() {
  //
  const auto shape = add_cube();
  // const auto shape = make_triangle();

  auto win = viewer::get_window3d("GJK View");
  auto geom = win->add_primitive<viewer::SimpleGeometry>();
  auto mesh_a = win->add_primitive<viewer::SimpleGeometry>();

  const auto visitor = [&win, &geom](const Simplex& s) {
    Vec4 color(1.0, 0.0, 0.2, 1.0);
    add_simplex(*geom, s, color);

    geom->flip();
    win->spin_until_step();
    geom->clear();
  };

  const Vec3 base_target(1.0, 3.0, 1.0);
  double d = 0.0;
  while (!win->should_close()) {
    d += 0.01;
    const SO3 rot = SO3::exp(Vec3(d, -d, std::exp(std::cos(d))));

    const Vec3 target = rot * base_target;

    const auto closest_point = find_closest_point(target, shape);

    mesh_a->add_point({target, Vec4(1.0, 1.0, 0.0, 1.0)});
    mesh_a->add_point({closest_point.point, Vec4(0.0, 1.0, 0.0, 1.0), 4.0});

    mesh_a->add_line({target, closest_point.point,
                      Vec4(closest_point.distance / 3.0, 1.0, 0.0, 0.8), 3.0});

    add_simplex(*mesh_a,
                shape.simplices[closest_point.simplex_index],
                Vec4(1.0, 0.0, 0.0, 1.0),
                7.0);

    mesh_a->flip();
    win->spin_until_step();
  }

  gjk(shape, base_target, visitor);
}

}  // namespace spatial
}  // namespace geometry

int main() {
  geometry::spatial::demo_intersection();
}