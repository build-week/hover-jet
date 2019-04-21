#pragma once

#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>
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
    std::string serial_number = "uninitialized";
    std::string v4l_path;
    int video_index = 0;
    Calibration calibration;
};

class CameraManager {
    public:
        CameraManager();
        Camera get_camera(const std::string& serial_number) const;
    private:
        void load_configs();
        std::optional<YAML::Node> read_YAML(const std::string& filepath) const;
        std::optional<Camera> parse_config(const YAML::Node& cfg) const;
        std::optional<int> follow_v4l_path(const std::string& path) const;
        std::map<std::string, Camera> camera_map_;
};

} // namespace jet