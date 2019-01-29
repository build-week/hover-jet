#include "block_transform.hh"
#include "features.hh"
#include "histogram_equalize.hh"
#include "images.hh"
#include "io.hh"
#include "pyramid.hh"

#include "util/clamp.hh"

#include "eigen.hh"
#include <iostream>
#include <opencv2/opencv.hpp>

using Vec2 = Eigen::Vector2f;

struct MappingContext {
  cv::Mat           rgb_image;
  std::vector<Vec2> points;
};

void visualize(const MappingContext &context) {
  cv::Mat visualization_image = context.rgb_image;

  for (const auto &point : context.points) {
    constexpr int    RADIUS = 1;
    const cv::Scalar COLOR(0, 255, 0);
    cv::circle(visualization_image, cv::Point(point.x(), point.y()), RADIUS, COLOR);
  }

  cv::imshow("Visualization", visualization_image);
}

cv::Mat normalize(const cv::Mat &grayscale_image) {
  // Compute a block normalization of an grayscale_image

  cv::Mat_<float> float_gs_image;
  grayscale_image.convertTo(float_gs_image, CV_32FC1);

  double min, max;
  cv::minMaxLoc(float_gs_image, &min, &max);

  return (float_gs_image - min) / (max - min);
}

std::vector<Vec2> extract_harris_features(const cv::Mat &image) {
  constexpr double RELATIVE_SCALE     = 1.0;
  constexpr double INV_RELATIVE_SCALE = 1.0 / RELATIVE_SCALE;

  cv::Mat scaled_image;
  cv::resize(image, scaled_image, cv::Size(0, 0), RELATIVE_SCALE, RELATIVE_SCALE);
  const cv::Mat normalized      = normalize(scaled_image);
  const float   mean            = cv::mean(normalized)[0];
  cv::Mat       mean_subtracted = normalized - mean;

  cv::imshow("Gombo", normalized - mean);

  float const *const pixel_ptr = (float *)mean_subtracted.data;

  std::vector<Vec2> locations;
  for (int col = 0; col < mean_subtracted.cols; ++col) {
    for (int row = 0; row < mean_subtracted.rows; ++row) {
      const int   index     = row * (mean_subtracted.cols) + col;
      const float pixel_val = pixel_ptr[index];

      constexpr float NORMALIZED_THRESHOLD = 0.05;
      if (pixel_val > NORMALIZED_THRESHOLD) {
        locations.emplace_back(col * INV_RELATIVE_SCALE, row * INV_RELATIVE_SCALE);
      }
    }
  }
  return locations;
}

int main() {
  const auto image_and_depth_locations = slam::get_both();

  for (const auto &location_pair : image_and_depth_locations) {
    const cv::Mat image = cv::imread(location_pair.image);
    const cv::Mat depth = cv::imread(location_pair.depth, CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    const cv::Mat_<float> normalized_gray_image = normalize(gray_image);

    constexpr int BLOCK_TRANSFORM_SIZE = 100;
    const cv::Mat equalized = slam::block_transform<BLOCK_TRANSFORM_SIZE>(gray_image, slam::histogram_equalize);

    cv::imshow("Equalized", equalized);

    const auto get_harris = [&](const cv::Mat &sub_img) {
      constexpr int    HARRIS_BLOCK_SIZE     = 2;
      constexpr int    HARRIS_K_SIZE         = 5;
      constexpr double HARRIS_FREE_PARAMETER = 0.0001;

      cv::Mat temp_harris_image;
      cornerHarris(sub_img, temp_harris_image, HARRIS_BLOCK_SIZE, HARRIS_K_SIZE, HARRIS_FREE_PARAMETER);
      return temp_harris_image;
    };

    const cv::Mat harris_image = slam::block_transform<BLOCK_TRANSFORM_SIZE>(normalize(equalized), get_harris);

    double min, max;
    cv::minMaxLoc(harris_image, &min, &max);

    std::cout << harris_image.size() << std::endl;
    std::cout << "MIN: " << min << std::endl;
    std::cout << "MAX: " << max << std::endl;
    cv::imshow("Determinants", harris_image);
    cv::imshow("Input", image);

    // const auto pts = extract_harris_features(harris_image);

    // MappingContext context;
    // context.points = pts;
    // context.rgb_image = image;
    // visualize(context);

    const int key = cv::waitKey(0);
    if (key == 113 || key == 1048689) {
      break;
    }
  }
}
