#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>

namespace slam {

template <int BLOCK_SIZE, typename Callable>
cv::Mat block_transform(const cv::Mat &input_image, const Callable &function) {
  // OpenCV API sucks
  cv::Mat output(input_image.size(), input_image.type());

  for (int start_col = 0; start_col < output.cols; start_col += BLOCK_SIZE) {
    for (int start_row = 0; start_row < output.rows; start_row += BLOCK_SIZE) {

      const int x_block_size = std::min(BLOCK_SIZE, output.cols - start_col);
      const int y_block_size = std::min(BLOCK_SIZE, output.rows - start_row);

      const auto rect = cv::Rect(start_col, start_row, x_block_size, y_block_size);
      const cv::Mat sub_image = input_image(rect).clone();

      function(sub_image).copyTo(output(rect));
    }
  }
  return output;
}
}
