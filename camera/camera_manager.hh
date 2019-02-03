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
    std::string v4l_path;
    int video_index;
    Calibration calibration;
};

class CameraManager {
    public:
        CameraManager();
        static Camera get_camera(int camera_number);
    private:
        std::map<int, Camera> camera_map_;
        void parse_config(YAML::Node cfg);
        int follow_v4l_path(std::string path);
};

} // namespace jet