
#include "out.hh"
#include "sophus.hh"

#include "viewer/primitives/plot.hh"
#include "viewer/window.hh"

#include "vision/block_transform.hh"
#include "vision/camera_model.hh"
#include "vision/features.hh"
#include "vision/render/render_from_pose.hh"

#include "eigen.hh"
#include <opencv2/opencv.hpp>

#include <iostream>

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

namespace slam {

// Quickly compute eigenvalues for a symmetric 2x2 matrix
// Note: The eigenvectors are always orthogonal. Isn't that fun?
//
// Largest eigenvalue first
inline Vec2 eig_symmetric2(const float xx, const float xy, const float yy) {
  const double t           = xx + yy;
  const double d           = (xx * yy) - (xy * xy);
  const double half_t      = t * 0.5;
  const double second_term = std::sqrt((half_t * half_t) - d);

  Vec2 vals(half_t + second_term, half_t - second_term);
  return vals[0] > vals[1] ? vals : vals.reverse();
}

cv::Mat gauss_blur(const cv::Mat &image, const double sigma) {
  cv::Mat blurred_image;
  cv::GaussianBlur(image, is_out(blurred_image), cv::Size(0, 0), sigma, sigma);
  return blurred_image;
}

// Compute hessians at every pixel in an image
//
// What's interesting about this?
//  - Edge information can be extracted from hessians with a dominant eigenvalue
//
cv::Mat compute_hessians(const cv::Mat &image) {
  constexpr double h      = 1.0;
  constexpr double h2_inv = (1.0 / (h * h));

  cv::Mat hessian(image.size(), CV_32FC1);
  hessian = cv::Mat::zeros(image.size(), CV_32FC1);
  // cv::Mat hessian(image.size(), CV_32FC3);

  std::vector<double> vals;
  for (int row = 1; row < image.rows - 1; ++row) {
    for (int col = 1; col < image.cols - 1; ++col) {
      const float ixx =
          h2_inv * (image.at<uchar>(row, col + 1) - 2.0 * image.at<uchar>(row, col) + image.at<uchar>(row, col - 1));
      const float iyy =
          h2_inv * (image.at<uchar>(row + 1, col) - 2.0 * image.at<uchar>(row, col) + image.at<uchar>(row - 1, col));

      const float ixy = 0.25 * h2_inv * (image.at<uchar>(row + 1, col + 1) - image.at<uchar>(row - 1, col + 1) -
                                         image.at<uchar>(row + 1, col - 1) + image.at<uchar>(row - 1, col - 1));

      vals.push_back(ixx + iyy);
      // const Vec2 eigvals = eig_symmetric2(ixx, ixy, iyy);
      const double det_hessian = ixx * iyy - (ixy * ixy);
      // hessian.at<float>(row, col) = eigvals(0) * eigvals(1);
      hessian.at<float>(row, col) = det_hessian;
      // }
    }
  }

  auto win2d   = viewer::get_window2d("Window A");
  auto plotter = std::make_shared<viewer::Plot>();
  plotter->add_histogram({vals, {100, true, -30.0, 30.0}, {0.0, 1.0, 0.0, 0.8}});
  win2d->add_primitive(plotter);
  // win2d->spin_until_step();
  win2d->clear();

  return hessian;
}

void run() {
  // - Discover features in a simulated image
  // - Attempt to estimate the camera pose from those features
  //     - Try RANSAC
  //     - Try Gauss-Newton pose optimization
  // - Normalied Cross Correlation alignment refinement
  // - Attempt to calibration

  // Then: 3D structure estimation (Mapping)
  // Object classification
  //
  const std::string image_path = "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";
  const cv::Mat     image_full = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat           image;
  cv::resize(image_full, is_out(image), cv::Size(0, 0), 0.5, 0.5);

  constexpr double  FX = 1600.0;
  constexpr double  FY = 1067.0;
  constexpr double  CX = 0.0;
  constexpr double  CY = 0.0;
  const CameraModel cam_model(FX, FY, CX, CY);

  for (int k = 1; k < 45; ++k) {
    SE3 image_from_camera;
    image_from_camera.translation() = Vec3(0.5, 0.5, -1000.0);
    image_from_camera.so3()         = SO3::exp(Vec3(0.0, k * 0.01, 0.0));

    const auto rendered_image = render::render_from_pose(image, cam_model, image_from_camera);
    cv::imshow("Re-rendered", rendered_image);

    const cv::Mat difference_of_gassuans = gauss_blur(rendered_image, 0.2) - gauss_blur(rendered_image, 0.8);

    const cv::Mat hess = compute_hessians(difference_of_gassuans);
    cv::imshow("blurred", difference_of_gassuans);

    const cv::Mat block_normalized_hessian =
        block_transform<150>(hess, [](const cv::Mat &mat) { return normalize(mat); });
    // cv::imshow("hessian", normalize(hess));
    cv::imshow("hessian", block_normalized_hessian);

    const int key = cv::waitKey(0);
    if (key == 113 || key == 1048689) {
      break;
    }
  }
}
}

int main() {
  slam::run();
}
