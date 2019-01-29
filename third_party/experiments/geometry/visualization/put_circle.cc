#include "geometry/visualization/put_circle.hh"

#include "geometry/rotation_to.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace geometry {
namespace visualization {
using Vec3 = Eigen::Vector3d;

void put_circle(viewer::SimpleGeometry& geo, const geometry::shapes::Circle& circle) {
  constexpr double DELTA_RAD = 0.01;

  const auto rot_real_from_circle = geometry::rotation_to(Vec3::UnitZ(), circle.u_normal);
  const SE3 world_from_circle = SE3(rot_real_from_circle, circle.center);

  viewer::Polygon polygon;
  polygon.outline = true;
  for (double s = 0.0; s <= M_PI * 2.0; s += DELTA_RAD) {
    const Vec3 pt_circle_frame = circle.radius * Vec3(std::cos(s), std::sin(s), 0.0);

    polygon.points.push_back(world_from_circle * pt_circle_frame);
  }
  geo.add_polygon(polygon);
}
}  // namespace visualization
}  // namespace geometry