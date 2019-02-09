//%deps(yaml-cpp)

#include "camera/camera_manager.hh"

namespace jet {

CameraManager::CameraManager() {
    const std::string& dir_name = "/jet/camera/cfg/";
    // directory iteration from http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
    DIR* dir_path = opendir(dir_name.c_str());
    if (dir_path == NULL) {
        std::string err = std::string("Unable to open ") + dir_name;
        throw std::runtime_error(err);
    }
    const struct dirent * dp;
    while ((dp = readdir(dir_path)) != NULL) {
        std::string filename = dp->d_name;
        if (filename == "." || filename == "..") continue;
        try {
            YAML::Node config = YAML::LoadFile(dir_name + filename);
            parse_config(config);
        } catch (YAML::BadFile e) {
            std::string err = std::string("Could not find YAML file ") + filename;
            throw std::runtime_error(err);
        }
    }
    // if (camera_map_.size() == 0) {
    //     std::string err = std::string("No camera configs read!");
    //     throw std::runtime_error(err);
    // }
}

void CameraManager::parse_config(YAML::Node cfg) {
    Camera camera;
    // path & video index
    camera.serial_number = cfg["serial_number"].as<std::string>();
    camera.v4l_path = "/dev/v4l/by-id/" + cfg["v4l_path"].as<std::string>();
    camera.video_index = follow_v4l_path(camera.v4l_path);
    if (camera.video_index == -1)
        return;
    // calibration values
    std::vector<double> camera_matrix_values = cfg["camera_matrix"].as<std::vector<double>>();
    camera.calibration.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_matrix_values[0]);
    std::vector<double> distortion_coefficient_values = cfg["distortion_coefficients"].as<std::vector<double>>();
    camera.calibration.distortion_coefficients = cv::Mat(1, 5, CV_64F, &distortion_coefficient_values[0]);

    camera_map_[camera.serial_number] = camera;
}

int CameraManager::follow_v4l_path(std::string path) {
    // http://pubs.opengroup.org/onlinepubs/9699919799/functions/readlink.html
    char buf[1024];
    ssize_t len;
    if ((len = readlink(path.c_str(), buf, sizeof(buf)-1)) != -1)
        buf[len] = '\0';
    else 
        return -1;
    return buf[len-1] - '0';
}

Camera CameraManager::get_camera(std::string serial_number) {
    CameraManager instance = CameraManager();
    return instance.camera_map_[serial_number];
}

} // namespace jet