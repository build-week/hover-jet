#pragma once

//%deps(opengl, opencv)

#include <atomic>
#include <mutex>

#include "eigen.hh"
#include "viewer/primitives/primitive.hh"

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace viewer {

class Image final : public Primitive {
 public:
  Image(const cv::Mat& image, double width_m = 1.0, double alpha = 0.7);
  Image(const Eigen::MatrixXd& image, double width_m = 1.0, double alpha = 0.7);

  void update_image(const cv::Mat& image);
  void update_image(const Eigen::MatrixXd& image);

  void draw() const override;

 private:
  void update_gl() const;

  cv::Mat image_;
  double width_m_ = 1.0;
  double  alpha_ = 0.7;

  mutable unsigned int      texture_id_;
  mutable bool              allocated_texture_ = false;
  mutable std::atomic<bool> to_update_{false};
  mutable std::mutex        draw_mutex_;
};
}  // namespace viewer
