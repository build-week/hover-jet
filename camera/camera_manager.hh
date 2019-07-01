#pragma once

#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>
#include <string>

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct Calibration {
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
};

struct Camera {
  std::string serial_number;
  std::string v4l_path;
  Calibration calibration;
};

class CameraManager {
 public:
  CameraManager();
  Camera get_camera(const std::string& serial_number) const;
  Calibration get_calibration(const std::string& serial_number) const;

 private:
  void load_configs();
  std::map<std::string, Camera> camera_from_serial_;
  std::map<std::string, Calibration> camera_calibration_from_serial_;
};

}  // namespace jet