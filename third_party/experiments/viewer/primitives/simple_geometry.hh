#pragma once

//%deps(opengl)

#include "viewer/primitives/simple_geometry_primitives.hh"

#include "geometry/ray.hh"
#include "geometry/tri_mesh.hh"

#include "viewer/primitives/primitive.hh"

#include "sophus.hh"

#include <map>
#include <mutex>
#include <vector>

namespace viewer {

class SimpleGeometry final : public Primitive {
 public:
  SimpleGeometry() = default;

  void draw() const override;

  void add_axes(const Axes &axes);

  void add_line(const Line &line);

  void add_ray(const Ray &ray);

  void add_ray(const geometry::Ray &ray,
               const double length = 1.0,
               const Eigen::Vector4d &color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));

  void add_polygon(const Polygon &polygon);

  void add_points(const Points &points);

  void add_point(const Point &point);

  void add_colored_points(const Points &points, const std::vector<double> &intensities);

  void add_points2d(const Points2d &points);

  void add_sphere(const Sphere &sphere);

  void add_box(const AxisAlignedBox &box);

  void add_plane(const Plane &plane);

  void add_triangle_mesh(const TriMesh &mesh);

  void clear();

  void flip();

  void flush();

 private:
  struct Primitives {
    std::vector<Axes> axes;
    std::vector<Line> lines;
    std::vector<Points> points;
    std::vector<Point> raw_points;
    std::vector<Points2d> points2d;
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;
    std::vector<Polygon> polygons;
    std::vector<ColoredPoints> colored_points;
    std::vector<TriMesh> tri_meshes;

    void clear() {
      axes.clear();
      lines.clear();
      points.clear();
      raw_points.clear();
      points2d.clear();
      spheres.clear();
      planes.clear();
      polygons.clear();
      colored_points.clear();
      tri_meshes.clear();
    }
  };

  // This is not great
  // TODO: Add a universal UUID -> displaylist map
  mutable std::map<std::size_t, int> mesh_displaylists_;

  Primitives back_buffer_;
  Primitives front_buffer_;

  mutable std::mutex draw_mutex_;
};
}  // namespace viewer
