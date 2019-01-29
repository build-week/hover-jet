#pragma once

#include <opencv2/opencv.hpp>

//
// Image pyramid
class Pyramid {
  static constexpr int LAYERS = 5;
  Pyramid(const cv::Mat &image) {
    //
    // Build the pyramid by successive downsampling
    //

    images_.push_back(image);
    for (int k = 1; k < LAYERS; ++k) {

      cv::Mat downsampled_image;
      cv::resize(images_[k - 1], downsampled_image, cv::Size(0, 0), 0.5, 0.5);

      images_.push_back(downsampled_image);
    }
  }

  const cv::Mat &level(int k) {
    assert(k < LAYERS);
    return images_[k];
  }

  const cv::Mat &bottom() {
    return images_[LAYERS - 1];
  }

  const cv::Mat &top() {
    return images_[0];
  }

private:
  std::vector<cv::Mat> images_;
};
