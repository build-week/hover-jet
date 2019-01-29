#include "histogram_equalize.hh"
#include "features.hh"

#include "eigen.hh"

namespace slam {
namespace {
constexpr int NUM_LEVELS = 256;
using FloatHistogram = Eigen::Matrix<float, NUM_LEVELS, 1>;
using IntHistogram = Eigen::Matrix<int, NUM_LEVELS, 1>;

FloatHistogram normalize_histogram(const IntHistogram &histogram, const int num_elements) {

  const double num_elements_inv = 1.0 / static_cast<double>(num_elements);
  FloatHistogram normalized_histogram;

  for (int k = 0; k < histogram.size(); ++k) {
    normalized_histogram(k) = histogram(k) * num_elements_inv;
  }
  return normalized_histogram;
}

FloatHistogram make_integral_histogram(const FloatHistogram &histogram) {
  FloatHistogram cdf;
  cdf(0) = 0.0;
  for (int k = 1; k < NUM_LEVELS; ++k) {
    cdf(k) = cdf(k - 1) + histogram(k);
  }

  return cdf;
}
}

cv::Mat histogram_equalize(const cv::Mat &image) {

  const float variance = compute_variance(image);
  if (variance < 500.0) {
    return image;
  }

  IntHistogram histogram = IntHistogram::Zero();

  const size_t num_pixels = image.total();
  for (size_t k = 0; k < num_pixels; ++k) {
    const uint8_t value = image.at<uchar>(k);
    ++histogram[value];
  }
  const FloatHistogram normalized_histogram = normalize_histogram(histogram, num_pixels);
  const FloatHistogram cdf = make_integral_histogram(normalized_histogram);

  cv::Mat new_image(image.size(), image.type());
  for (size_t k = 0; k < num_pixels; ++k) {
    const uchar value = image.at<uchar>(k);

    constexpr double NUM_LEVELS_M_1 = NUM_LEVELS - 1;
    const uchar corrected_value = NUM_LEVELS_M_1 * cdf(value);
    new_image.at<uchar>(k) = corrected_value;
  }

  return new_image;
}
}
