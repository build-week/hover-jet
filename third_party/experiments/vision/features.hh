#pragma once

#include <opencv2/opencv.hpp>

namespace slam {

cv::Mat normalize(const cv::Mat &image);

cv::Mat convolve(const cv::Mat &image, const cv::Mat &kernel);

cv::Mat ncc(const cv::Mat &image, const cv::Mat &kernel);

float compute_variance(const cv::Mat &image);
}
