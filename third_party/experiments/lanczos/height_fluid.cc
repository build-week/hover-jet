#include "eigen.hh"

#include "lanczos/eulerian_height_fluid.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace lanczos {
using Mat = Eigen::MatrixXd;
using Vec3 = Eigen::Vector3d;

Mat generate_terrain(int size) {
  Mat terrain_height = Mat::Zero(size, size);
  return terrain_height;
}

void visualize() {
  const auto view = viewer::get_window3d("Fluid Visualization");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
          Eigen::Vector3d::Zero()));
  view->clear();
  view->set_continue_time_ms(5);

  const double scale = 1.0;
  const int size = 500;

  EulerianHeightFluid system(size, scale);
  viewer::Points ground_points;

  // const Mat& terrain = system.terrain_field();
  // for (int x = 0; x < terrain.rows(); ++x) {
  //   for (int y = 0; y < terrain.cols(); ++y) {
  //     const double h = terrain(x, y);
  //     const Vec3 terrain_pt(x * scale, y * scale, h);
  //     ground_points.points.push_back(terrain_pt);
  //   }
  // }

  const auto fluid_geo = view->add_primitive<viewer::SimpleGeometry>();

  for (int k = 0; k < 10000; ++k) {
    const Mat heights = system.height_field();

    const size_t count = size * size;
    viewer::Points points;
    points.points.reserve(count);
    std::vector<double> intensities;
    intensities.reserve(count);
    points.size = scale * 5.0;

    for (int x = 0; x < heights.rows(); ++x) {
      for (int y = 0; y < heights.cols(); ++y) {
        const double h = heights(x, y);
        const Vec3 position(x * scale, y * scale, h);
        points.points.push_back(position);

        const double v_x = system.v_x()(x, y);
        const double v_y = system.v_y()(x, y);

        const double norm_v = std::hypot(v_x, v_y);

        // intensities.push_back(norm_v);
        intensities.push_back(h - 23.0);
      }
    }

    fluid_geo->add_colored_points(points, intensities);
    fluid_geo->flip();

    // const auto ground_geo = view->add_primitive<viewer::SimpleGeometry>();
    // ground_geo->add_points(ground_points);

    for (int iter = 0; iter < 10; ++iter) {
      system.simulate(0.001);
    }
    view->spin_until_step();
  }
}
}  // namespace lanczos

int main() {
  lanczos::visualize();
}