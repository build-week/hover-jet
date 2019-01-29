#include "image_align.hh"

#include "eigen_helpers.hh"
#include "out.hh"
#include "util/clamp.hh"

#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/image_frame.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include "eigen.hh"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <ostream>

using Vec2   = Eigen::Vector2d;
using Vec3   = Eigen::Vector3d;
using Vec32i = Eigen::Matrix<uint8_t, 32, 1>;

constexpr double DISTANCE = 500.0;

namespace slam {

std::vector<Vec2> view_object_from_pose(const CameraModel&       cam_model,
                                        const std::vector<Vec3>& points_object_frame,
                                        const SE3&               camera_from_object) {
  std::vector<Vec2> projected_points(points_object_frame.size());
  for (size_t k = 0; k < points_object_frame.size(); ++k) {
    projected_points[k] = cam_model.project(camera_from_object * points_object_frame[k]);
  }
  return projected_points;
}

std::vector<Vec32i> eigenize_descriptors(const cv::Mat& descriptors) {
  std::vector<Vec32i> eigen_descriptors(descriptors.rows);
  for (int k = 0; k < descriptors.rows; ++k) {
    for (int j = 0; j < descriptors.cols; ++j) {
      eigen_descriptors[k](j) = descriptors.at<uint8_t>(k, j);
    }
  }
  return eigen_descriptors;
}

std::vector<Vec32i> compute_orb_features(const cv::Mat& image, Out<std::vector<cv::KeyPoint>> key_points) {
  constexpr int   N_FEATURES     = 500;
  constexpr float SCALE_FACTOR   = 1.2f;
  constexpr int   N_LEVELS       = 8;
  constexpr int   EDGE_THRESHOLD = 12;
  constexpr int   FIRST_LEVEL    = 0;
  // const cv::ORB   orb_detector(N_FEATURES, SCALE_FACTOR, N_LEVELS, EDGE_THRESHOLD, FIRST_LEVEL);
  const auto detector = cv::ORB::create(N_FEATURES, SCALE_FACTOR, N_LEVELS, EDGE_THRESHOLD, FIRST_LEVEL);

  cv::Mat descriptors;
  detector->compute(image, is_out(*key_points), is_out(descriptors));
  return eigenize_descriptors(descriptors);
}

void estimate_camera_pose(const CameraModel&       model,
                          const std::vector<Vec2>& observed_pts,
                          const std::vector<Vec3>& calibration_pts) {
  if (observed_pts.size() < 5u) {
    return;
  }

  ImageAligner              aligner;
  const Sophus::SE3<double> initial;
  const auto                result             = aligner.standard_align(model, initial, observed_pts, calibration_pts);
  const SE3                 camera_from_object = result.delta;

  std::cout << camera_from_object.log().transpose() << std::endl;

  auto win  = viewer::get_window3d("Window A");
  auto geom = std::make_shared<viewer::SimpleGeometry>();
  win->add_primitive(geom);

  if (result.success) {
    geom->add_axes({camera_from_object});

    for (const Vec2& pt : observed_pts) {
      const geometry::Ray ray_camera_frame = model.unproject(pt);
      const geometry::Ray ray_object_frame = camera_from_object.inverse() * ray_camera_frame;
      geom->add_ray({ray_object_frame.origin, ray_object_frame.direction, 3500.0, Eigen::Vector4d(0.2, 0.7, 0.0, 0.6)});
    }
  }
  win->spin_until_step();
}

void simulate_views(const std::vector<cv::KeyPoint>& calibration_key_points) {
  constexpr double  FX = 1600.0;
  constexpr double  FY = 1067.0;
  constexpr double  CX = -1600 * 0.5;
  constexpr double  CY = -1067 * 0.5;
  const CameraModel cam_model(FX, FY, CX, CY);

  // Simulate a view
  std::vector<Vec3> object_points;
  for (const auto& pt : calibration_key_points) {
    // Using "distance" seriously improves conditioning
    // --> TODO(jake): Optimize camera_from_initial instead of camera_from_object
    //                 to eliminate lever arm effect
    object_points.emplace_back(pt.pt.x, pt.pt.y, DISTANCE);
  }

  for (int k = 0; k < 6; ++k) {
    const SE3 camera_from_object = SE3::exp(jcc::vstack(Vec3(k * 100, 200 * k, 300.0), Vec3(0.0, 0.0, k * 0.05)));
    std::cout << camera_from_object.log().transpose() << std::endl;
    const std::vector<Vec2> observed_points = view_object_from_pose(cam_model, object_points, camera_from_object);
    for (size_t k = 0; k < object_points.size(); ++k) {
      std::cout << observed_points[k].transpose() << " : " << (camera_from_object * object_points[k]).transpose()
                << std::endl;
    }
    estimate_camera_pose(cam_model, observed_points, object_points);
  }
}

// Weakly estimate pose

// Attempt ncc or ssd (Requires undistortion, no?)

// Strongly estimate pose

// TODO: Undistort
}

int main() {
  const std::string calibration_image_filename =
      "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";

  const cv::Mat calibration_image_color = cv::imread(calibration_image_filename);
  std::cout << calibration_image_color.size() << std::endl;
  cv::Mat calibration_image;
  cv::cvtColor(calibration_image_color, is_out(calibration_image), CV_BGR2GRAY);

  auto win = viewer::get_window3d("Window A");
  {
    auto calibration_img_primitive = std::make_shared<viewer::Image>(calibration_image_color, 1.0);
    auto calibration_img_primitive_frame =
        std::make_shared<viewer::Frame>(SE3(SO3(), Vec3(0.0, 0.0, DISTANCE + 0.1)));
    calibration_img_primitive_frame->add_primitive(calibration_img_primitive);
    win->add_primitive(calibration_img_primitive_frame);
  }

  std::vector<cv::KeyPoint> calibration_key_points;
  const auto calibration_descriptors = slam::compute_orb_features(calibration_image, out(calibration_key_points));
  // slam::try_align(calibration_key_points, calibration_key_points, calibration_image_color);
  slam::simulate_views(calibration_key_points);
}