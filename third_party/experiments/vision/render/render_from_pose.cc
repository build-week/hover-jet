#include "render_from_pose.hh"

#include <opencv2/opencv.hpp>

// todo
#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

namespace slam {
namespace render {
using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

namespace {

// Take a non-integer image point in ref_image, and get a pixel
// TODO: interpolate
// For now, it just rounds the coordinates down
uint8_t sample_image(const cv::Mat& ref_image, const Vec2& image_point) {
  const int icol = static_cast<int>(image_point.x());
  const int irow = static_cast<int>(image_point.y());

  if (icol < ref_image.cols && irow < ref_image.rows && icol >= 0 && irow >= 0) {
    return ref_image.at<uint8_t>(irow, icol);
  } else {
    return 0u;
  }
}
}

cv::Mat render_from_pose(const cv::Mat& ref_image, const CameraModel& cam_model, const SE3& image_from_camera) {
  cv::Mat rendered_image(ref_image.size(), ref_image.type());

  const Vec3   image_normal_camera_frame = image_from_camera.so3().inverse() * Vec3::UnitZ();
  const double image_distance            = image_from_camera.inverse().translation().dot(image_normal_camera_frame);

  for (int row = 0; row < ref_image.rows; ++row) {
    for (int col = 0; col < ref_image.cols; ++col) {
      const Vec2          image_point(col, row);
      const geometry::Ray ray_image_frame = cam_model.unproject(image_point);
      const double        estimated_distance_along_ray_m =
          image_distance / (ray_image_frame.direction.dot(image_normal_camera_frame));
      const Vec3 plane_point_camera_frame = ray_image_frame(estimated_distance_along_ray_m);
      const Vec3 plane_point_image_frame  = image_from_camera * plane_point_camera_frame;
      rendered_image.at<uint8_t>(row, col) = sample_image(ref_image, plane_point_image_frame.head<2>());
    }
  }

  constexpr bool DRAW_DEBUG_DATA = false;
  if (DRAW_DEBUG_DATA) {
    auto viewer = viewer::get_window3d("America's Favorite Visualization Tool");
    auto geom   = std::make_shared<viewer::SimpleGeometry>();
    auto image  = std::make_shared<viewer::Image>(ref_image);
    auto frame  = std::make_shared<viewer::Frame>(image_from_camera);
    frame->add_primitive(image);

    viewer->add_primitive(geom);
    viewer->add_primitive(frame);
    viewer->spin_until_step();
  }

  return rendered_image;
}
}
}
