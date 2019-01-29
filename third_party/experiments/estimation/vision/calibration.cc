#include "estimation/vision/calibration.hh"

namespace estimation {
namespace vision {

constexpr int BOARD_WIDTH = 9;
constexpr int BOARD_HEIGHT = 6;
constexpr float SQUARE_SIZE = .034;

void CalibrationManager::add_camera_image(const cv::Mat image) {
  all_camera_images_.push_back(image.clone());
}

int CalibrationManager::num_images_collected() const{
  return all_camera_images_.size();
}

CameraIntrinsics CalibrationManager::calibrate() const{
  // based on
  // https://github.com/sourishg/stereo-calibration/blob/master/calib_intrinsic.cpp
  assert(all_camera_images_.size() > 0);

  const auto board_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);

  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;

  std::vector<cv::Point3f>
      obj;  // simply a description of the board's intersection points
  for (int i = 0; i < BOARD_HEIGHT; i++) {
    for (int j = 0; j < BOARD_WIDTH; j++) {
      obj.push_back(cv::Point3f(static_cast<float>(j * SQUARE_SIZE),
                                static_cast<float>(i * SQUARE_SIZE), 0));
    }
  }

  for (auto const& image : all_camera_images_) {
    cv::Mat gray;
    std::vector<cv::Point2f> corners;

    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
    bool found =
        cv::findChessboardCorners(gray, board_size, corners,
                                  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (found) {
      std::cout << "-";
      cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,
                                    30, 0.001));
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
  std::cout << "\\" << std::endl;
  std::cout << "Calibrating" << std::endl;

  cv::Mat K;
  cv::Mat D;
  std::vector<cv::Mat> rvecs, tvecs;
  int flag = 0;
  // flag |= CV_CALIB_FIX_K4;
  // flag |= CV_CALIB_FIX_K6;
  cv::calibrateCamera(object_points, image_points, all_camera_images_[0].size(), K, D,
                      rvecs, tvecs, flag);
  std::cout << "Done calibrating" << std::endl;

  CameraIntrinsics result;
  result.K = K;
  result.D = D;
  return result;
}

}  // namespace vision
}  // namespace estimation