#pragma once

#include <opencv2/opencv.hpp>

namespace slam {
cv::Mat histogram_equalize(const cv::Mat &image);
}
