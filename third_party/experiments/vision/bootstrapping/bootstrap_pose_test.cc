#include "bootstrap_pose.hh"

#include "testing/gtest.hh"

namespace slam {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

class BootstrapPoseTest : public ::testing::Test {
 protected:
  BootstrapPoseTest() {
    constexpr double FX = 100;
    constexpr double FY = 100;
    constexpr double CX = 50.0;
    constexpr double CY = 50.0;
    cam_model_          = CameraModel(FX, FY, CX, CY);
  }

  std::vector<Vec3> generate_object(const int num_pts) {
    std::vector<Vec3> points_object_frame(num_pts);

    for (int k = 0; k < num_pts; ++k) {
      const Vec3 pt_object_frame = Vec3(100.0, 100.0, 1.0).asDiagonal() * (Vec3::Random() - Vec3(0.5, 0.5, 0.0));
      points_object_frame[k]     = pt_object_frame;
    }
    return points_object_frame;
  }

  std::vector<Vec2> view_object_from_pose(const std::vector<Vec3>& points_object_frame, const SE3& camera_from_object) {
    std::vector<Vec2> projected_points(points_object_frame.size());
    for (size_t k = 0; k < points_object_frame.size(); ++k) {
      projected_points[k] = cam_model_.project(camera_from_object * points_object_frame[k]);
    }
    return projected_points;
  }

  CameraModel cam_model_ = CameraModel(Eigen::Matrix3d::Identity());
};

TEST_F(BootstrapPoseTest, can_bootstrap) {
  //
  // Setup
  //

  const SE3 camera_a_from_object = SE3(SO3(), Vec3(0.0, 0.0, 15.0));
  const SE3 camera_b_from_object = SE3(SO3::exp(Vec3(0.0, -0.5, 0.0)), Vec3(5.0, 0.0, 15.0));
  // const SE3 camera_b_from_object = SE3(SO3::exp(Vec3(0.0, 0.0, 0.00)), Vec3(5.0, 0.0, -5.0));

  const auto object_pts = generate_object(20);

  const std::vector<Vec2> image_points_a  = view_object_from_pose(object_pts, camera_a_from_object);
  const std::vector<Vec2> image_points_b  = view_object_from_pose(object_pts, camera_b_from_object);
  const SE3               camera_b_from_a = camera_b_from_object * camera_a_from_object.inverse();

  compute_nonmetric_pose(image_points_a, image_points_b, cam_model_, camera_b_from_a);
}
}