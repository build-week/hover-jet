//%deps(yaml-cpp)

#include "camera/calibration_manager.hh"

namespace jet {

CalibrationManager::CalibrationManager() {
    const std::string& dir_name = "/jet/camera/cfg/";
    // directory iteration from http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
    DIR* dir_path = opendir(dir_name.c_str());
    if (dir_path == NULL) {
        std::string err = std::string("Unable to open ") + dir_name;
        throw std::runtime_error(err);
    }
    struct dirent * dp;
    while ((dp = readdir(dir_path)) != NULL) {
        std::string filename = dp->d_name;
        if (filename == "." || filename == "..") continue;
        std::cout << "reading file " << filename << std::endl;
        try {
            YAML::Node config = YAML::LoadFile(dir_name + filename);
            parse_config(config);
        } catch (YAML::BadFile e) {
            std::string err = std::string("Could not find YAML file ") + filename;
            throw std::runtime_error(err);
        }
    }
    if (camera_calibration_dict_.size() == 0) {
        std::string err = std::string("No camera configs read!");
        throw std::runtime_error(err);
    }
}

void CalibrationManager::parse_config(YAML::Node cfg) {
    CameraCalibration cal;
    int camera_number = cfg["camera_number"].as<int>();
    std::vector<double> camera_matrix_values = cfg["camera_matrix"].as<std::vector<double>>();
    cal.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_matrix_values[0]);
    std::vector<double> distortion_coefficient_values = cfg["distortion_coefficients"].as<std::vector<double>>();
    cal.distortion_coefficients = cv::Mat(1, 5, CV_64F, &distortion_coefficient_values[0]);

    camera_calibration_dict_[camera_number] = cal;
}

CameraCalibration CalibrationManager::get_camera_calibration(int camera_number) {
    return camera_calibration_dict_[camera_number];
}

} // namespace jet