#include "eigen.hh"

#include "geometry/visualization/visualize_nullspace.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geometry {
namespace visualization {

using Vec3 = Eigen::Vector3d;

void go() {
  const auto view = viewer::get_window3d("Mr. Demo");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  const double level_set = 5.0;
  const auto func = [level_set](const Eigen::VectorXd& x) {
    Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
    H(1, 1) = 2.0;
    return x.dot(H * x) - level_set;
    // return x.dot(Eigen::Vector2d(std::sin(x(0)), 0.0)) - 1.0;
    // return std::sin(x(0)) + (x(1) * x(1)) - 3.0;
  };

  visualize_nullspace(*geo, func, Eigen::VectorXd::Ones(2));

  geo->flip();
  view->spin_until_step();
}

}  // namespace visualization
}  // namespace geometry
int main() {
  geometry::visualization::go();
}