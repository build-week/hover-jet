#pragma once

#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <string>

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct CameraCalibration {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
};

class CalibrationManager {
    public:
        CalibrationManager();
        CameraCalibration get_camera_calibration(int camera_number);
    private:
        std::map<int, CameraCalibration> camera_calibration_dict_;
        void parse_config(YAML::Node cfg);
};

} // namespace jet