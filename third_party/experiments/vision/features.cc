#include "features.hh"

#include <iostream>

namespace slam {

cv::Mat normalize(const cv::Mat &grayscale_image) {
  // Compute a block normalization of an grayscale_image

  cv::Mat_<float> float_gs_image;
  grayscale_image.convertTo(float_gs_image, CV_32FC1);

  double min, max;
  cv::minMaxLoc(float_gs_image, &min, &max);

  const cv::Mat normalized_image = (float_gs_image - min) / (max - min);
  const float mean = cv::mean(normalized_image)[0];
  return normalized_image - mean;
}

cv::Mat ncc(const cv::Mat &image, const cv::Mat &kernel) {
  cv::Mat result(image.size(), CV_32FC1);
  cv::Mat image_f;
  image.convertTo(image_f, CV_32FC1);
  cv::Mat kernel_f;
  kernel.convertTo(kernel_f, CV_32FC1);

  const cv::Mat kernel_meaned = kernel - cv::mean(kernel)[0];
  const float kernel_sigma = std::sqrt(compute_variance(kernel));

  //
  // Do the actual computation
  //

  const int col_kernel_center = kernel.cols / 2;
  const int row_kernel_center = kernel.rows / 2;

  for (int col = 0; col + kernel.cols < image.cols; ++col) {
    for (int row = 0; row + kernel.rows < image.rows; ++row) {
      const auto rect = cv::Rect(col, row, kernel.cols, kernel.rows);
      const cv::Mat sub_image = image_f(rect);
      const cv::Mat hadamard_product = sub_image.mul(kernel_f);
      const float sum = cv::sum(hadamard_product)[0];
      result.at<float>(col + col_kernel_center, row + row_kernel_center) = sum;
    }
  }
  return result;
}

cv::Mat convolve(const cv::Mat &image, const cv::Mat &kernel) {
  //
  // Crappiest possible implementation first
  // TODO: something that resembles integral image
  //

  cv::Mat result(image.size(), CV_32FC1);
  cv::Mat image_f;
  image.convertTo(image_f, CV_32FC1);
  cv::Mat kernel_f;
  kernel.convertTo(kernel_f, CV_32FC1);

  for (int col = 0; col + kernel.cols < image.cols; ++col) {
    for (int row = 0; row + kernel.rows < image.rows; ++row) {
      const auto rect = cv::Rect(col, row, kernel.cols, kernel.rows);
      const cv::Mat sub_image = image_f(rect);
      const cv::Mat hadamard_product = sub_image.mul(kernel_f);
      const float sum = cv::sum(hadamard_product)[0];
      result.at<float>(col, row) = sum;
    }
  }
  return result;
}

float compute_variance(const cv::Mat &image) {
  float mean = cv::mean(image)[0];
  double sum_sq = 0.0;

  for (int col = 0; col < image.cols; ++col) {
    for (int row = 0; row < image.rows; ++row) {
      const double pixel_val = image.at<uchar>(row, col);
      const double error = pixel_val - mean;
      sum_sq += error * error;
    }
  }

  const double num_elements = static_cast<double>(image.total());
  const double inv_num_elements = 1.0 / num_elements;
  const double variance = sum_sq * inv_num_elements;
  return variance;
}
}
