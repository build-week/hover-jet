#include "eigen.hh"

#include "geometry/intersection/sphere_plane.hh"
#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"
#include "geometry/visualization/put_circle.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace intersection {

using Vec3 = Eigen::Vector3d;

void go() {
  const auto view = viewer::get_window3d("Sphere-Plane");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  constexpr double dt = 0.1;
  for (double t = 0.0; t < 100.0; t += dt) {
    const shapes::Plane ground{Vec3::UnitZ(), 0.0};
    geo->add_plane({ground});

    const shapes::Sphere sphere{Vec3(1.0, 1.0, std::cos(t)), 1.0};
    geo->add_sphere({sphere.center, sphere.radius});

    const auto intersection = sphere_plane_intersection(sphere, ground);

    if (intersection) {
      visualization::put_circle(*geo, *intersection);
    }

    geo->flip();
    view->spin_until_step();
  }
}

}  // namespace intersection
}  // namespace geometry
int main() {
  geometry::intersection::go();
}