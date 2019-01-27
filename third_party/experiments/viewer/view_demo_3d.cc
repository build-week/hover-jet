
#include "viewer/primitives/box.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/plot.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "viewer/window_3d.hh"

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/raycaster.hh"
#include "geometry/spatial/sphere_volume.hh"
#include "geometry/spatial/triangle_volume.hh"

#include "eigen_helpers.hh"
#include "sophus.hh"

#include <iostream>
#include <memory>
#include <thread>
#include <utility>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace viewer {

Eigen::MatrixXd make_quadratic_thing() {
  using Vec2 = Eigen::Vector2d;

  constexpr double SCALE = 0.01;
  Eigen::MatrixXd values(500, 500);
  Eigen::Matrix2d hessian = Eigen::Matrix2d::Identity();
  hessian(0, 0) = 2.0;
  hessian(1, 0) = -2.0;
  const Vec2 origin = Vec2(values.rows() / 2, values.cols() / 2) * SCALE;

  for (int row = 0; row < values.rows(); ++row) {
    for (int col = 0; col < values.cols(); ++col) {
      const Vec2 point = Vec2(row * SCALE, col * SCALE) - origin;
      values(row, col) = point.transpose() * (hessian * point);
    }
  }
  return values;
}

Eigen::MatrixXd mat_from_cv(const cv::Mat &image) {
  constexpr double INV_255 = 1.0 / 255.0;
  Eigen::MatrixXd mat(image.rows, image.cols);
  for (int row = 0; row < image.rows; ++row) {
    for (int col = 0; col < image.cols; ++col) {
      mat(row, col) = static_cast<double>(image.at<uint8_t>(row, col, 0)) * INV_255;
    }
  }
  return mat;
}

void run() {
  auto win = get_window3d("Window A");

  const std::string file_path = "/home/jacob/repos/experiments/data/test_stuff.stl";
  const auto tri = *geometry::import::read_stl(file_path);

  const std::string godzilla_image_filename = "/home/jacob/repos/slam/data/calibration/godzilla.jpg";
  const cv::Mat godzilla_image_color = cv::imread(godzilla_image_filename);
  const std::string calibration_image_filename =
      "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";
  const cv::Mat calibration_image_color = cv::imread(calibration_image_filename);

  const auto scene_geometry = std::make_shared<SimpleGeometry>();
  const auto lidar_geometry = std::make_shared<SimpleGeometry>();
  const auto cfg = geometry::spatial::build_raycaster_config();
  geometry::spatial::RayCaster ray_caster(cfg);

  std::cout << "Adding primitives" << std::endl;
  win->add_primitive(scene_geometry);
  win->add_primitive(lidar_geometry);

  scene_geometry->add_axes({SE3()});

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    scene_geometry->add_line({tri.triangles[k].vertices[0], tri.triangles[k].vertices[1]});
    scene_geometry->add_line({tri.triangles[k].vertices[1], tri.triangles[k].vertices[2]});
    scene_geometry->add_line({tri.triangles[k].vertices[2], tri.triangles[k].vertices[0]});

    const auto tri_volume = std::make_shared<geometry::spatial::TriangleVolume>(tri.triangles[k].vertices);
    ray_caster.add_volume(tri_volume);
  }

  scene_geometry->add_sphere({Vec3(5.0, 1.0, 1.0), 3.0});

  scene_geometry->add_box({Vec3(1.0, 1.0, 1.0), Vec3(2.0, 2.0, 3.0), Vec4(1.0, 0.2, 0.2, 0.6)});
  scene_geometry->add_box({Vec3(1.0, 1.0, 1.0), Vec3(2.0, 2.0, 2.0), Vec4(0.0, 1.0, 0.2, 0.6)});
  scene_geometry->flip();

  const auto sphere = std::make_shared<geometry::spatial::SphereVolume>(Vec3(5.0, 1.0, 1.0), 3.0);
  ray_caster.add_volume(sphere);

  for (double t = 0.0; t < 200.0; t += 0.01) {
    const auto world_from_caster = (SE3::exp(jcc::vstack(Vec3(0.0, 0.0, 0.0), Vec3(0.0, t, 0.0))));
    const auto distances = ray_caster.cast_rays(world_from_caster);

    lidar_geometry->clear();

    for (size_t k = 0; k < ray_caster.config().rays.size(); ++k) {
      const auto ray = world_from_caster * ray_caster.config().rays[k];
      if (distances[k] < 1000.0) {
        lidar_geometry->add_ray({ray.origin, ray.direction}, distances[k], Vec4(0.0, 1.0, 0.0, 0.5));
      } else {
        lidar_geometry->add_ray({ray.origin, ray.direction}, 10.0, Vec4(1.0, 0.0, 0.0, 0.25));
      }
    }
    lidar_geometry->flip();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  std::cout << "Done" << std::endl;
}  // namespace viewer
}  // namespace viewer

int main() {
  viewer::run();
}
