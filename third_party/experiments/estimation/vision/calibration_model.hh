#pragma once
#include <opencv2/opencv.hpp>

namespace estimation {
namespace vision {

class Calibration {
 public:
  Calibration(const cv::Mat& projection_matrix, const cv::Mat& distortion_coefficients) {
    projection_matrix_ = projection_matrix;
    distortion_coefficients_ = distortion_coefficients;
  }

  cv::Mat cv_projection() {
    return camera_matrix_;
  }

  cv::Mat cv_distortion() {
    return distortion_coefficients_;
  }

 private:
  cv::Mat projection_matrix_;
  cv::Mat distortion_coefficients_;
};
/*
  const cv::Mat camera_matrix =
      (cv::Mat1d(3, 3) << 265.9604351267379, 0, 323.3841822012849, 0, 302.7743119963964,
       177.7795703229708, 0, 0, 1);
  const cv::Mat distortion_coefficients = (cv::Mat1d(1, 5) << -0.0881294556831833,
                                           0.08627577358744372,
                                           -0.006574803742454203,
                                           0.01200680448589873,
                                           -0.02887266477084746);
*/

}  // namespace vision
}  // namespace estimation
