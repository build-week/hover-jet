#pragma once

#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <string>

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

struct Calibration {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
};
struct Camera {
    std::string serial_number = "CA38DB5E";
    std::string v4l_path;
    int video_index = 0;
    Calibration calibration;
};

class CameraManager {
    public:
        CameraManager();
        static Camera get_camera(std::string serial_number);
    private:
        std::map<std::string, Camera> camera_map_;
        void parse_config(YAML::Node cfg);
        int follow_v4l_path(std::string path);
};

} // namespace jet