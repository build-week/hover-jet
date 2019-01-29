#include "histogram_equalize.hh"
#include "block_transform.hh"

#include "testing/gtest.hh"

#include <string>

namespace slam {

TEST(HistogramEqualize, equalizes) {
  const std::string image_path = "/home/jacob/repos/slam/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png";
  const cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);

  cv::imshow("image", image);
  cv::waitKey(0);

  const cv::Mat equalized = histogram_equalize(image);
  cv::imshow("image", equalized);
  cv::waitKey(0);

  const cv::Mat block_equalized = block_transform<50>(image, histogram_equalize);
  cv::imshow("image", block_equalized);
  cv::waitKey(0);
}
}
