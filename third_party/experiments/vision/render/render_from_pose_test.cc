#include "render_from_pose.hh"

#include "testing/gtest.hh"

#include "out.hh"
#include "sophus.hh"

#include "vision/camera_model.hh"

#include "eigen.hh"
#include <opencv2/opencv.hpp>

namespace slam {
namespace render {

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

class RenderFromPoseTest : public ::testing::Test {
 protected:
  RenderFromPoseTest() {
    constexpr double FX = 1600.0;
    constexpr double FY = 1067.0;
    constexpr double CX = 0.0;
    constexpr double CY = 0.0;

    cam_model_ = CameraModel(FX, FY, CX, CY);
  }
  CameraModel cam_model_ = CameraModel(Eigen::Matrix3d::Identity());
};

TEST_F(RenderFromPoseTest, render_from_pose) {
  const std::string image_path = "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";
  const cv::Mat     image_full = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat           image;
  cv::resize(image_full, image, cv::Size(0, 0), 0.5, 0.5);

  SE3 image_from_camera;
  for (int k = 1; k < 10; ++k) {
    image_from_camera.translation() = Vec3(0.5, 0.5, -1000.0);
    image_from_camera.so3()         = SO3::exp(Vec3(0.0, k * 0.1, 0.0));

    const auto rendered_image = render_from_pose(image, cam_model_, image_from_camera);
    cv::imshow("Re-rendered", rendered_image);
    cv::waitKey(0);
  }
}
}
}