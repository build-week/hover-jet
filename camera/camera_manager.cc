//%deps(yaml-cpp)

#include "camera/camera_manager.hh"

namespace jet {

CameraManager::CameraManager() {
  load_configs();
  if (camera_map_.size() == 0) {
    std::string err = std::string("No camera configs read!");
    throw std::runtime_error(err);
  }
}

void CameraManager::load_configs() {
  // directory iteration from http://www.martinbroadhurst.com/list-the-files-in-a-directory-in-c.html
  DIR* dir_path = opendir(config_dir.c_str());
  if (dir_path == NULL) {
    std::string err = std::string("Unable to open ") + config_dir;
    throw std::runtime_error(err);
  }
  const struct dirent* dp;
  while ((dp = readdir(dir_path)) != NULL) {
    std::string filename = dp->d_name;
    if (filename == "." || filename == ".."){ 
      continue;
    }
    std::optional<YAML::Node> config = read_YAML(config_dir + filename);
    std::optional<Camera> camera = parse_config(*config);
    if (camera) {
      camera_map_[camera->serial_number] = *camera;
    }
  }
}

std::optional<YAML::Node> CameraManager::read_YAML(std::string filepath) {
  try {
    return YAML::LoadFile(filepath);
  } catch (YAML::BadFile e) {
    std::string err = std::string("Could not find YAML file ") + filepath;
    throw std::runtime_error(err);
  }
}

std::optional<Camera> CameraManager::parse_config(YAML::Node cfg) {
  Camera camera;
  // path & video index
  camera.serial_number = cfg["serial_number"].as<std::string>();
  camera.v4l_path = "/dev/v4l/by-id/" + cfg["v4l_path"].as<std::string>();
  camera.video_index = follow_v4l_path(camera.v4l_path);
  if (camera.video_index == -1) {
    return std::nullopt;
  }
  // calibration values
  std::vector<double> camera_matrix_values = cfg["camera_matrix"].as<std::vector<double>>();
  camera.calibration.camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_values.data()).clone();
  std::vector<double> distortion_coefficient_values = cfg["distortion_coefficients"].as<std::vector<double>>();
  camera.calibration.distortion_coefficients = cv::Mat(1, 5, CV_64F, distortion_coefficient_values.data()).clone();
  return camera;
}

int CameraManager::follow_v4l_path(std::string path) {
  // http://pubs.opengroup.org/onlinepubs/9699919799/functions/readlink.html
  char buf[1024];
  ssize_t len;
  if ((len = readlink(path.c_str(), buf, sizeof(buf) - 1)) != -1) {
    buf[len] = '\0';
  }
  else {
    return -1;
  }
  return buf[len - 1] - '0';
}

Camera CameraManager::get_camera(std::string serial_number) {
  if (camera_map_.count(serial_number) == 0) {
    std::string err = std::string(
        "The camera config passed to start the camera BQ specified "
        "a serial number that doesn't match the camera we have plugged in.");
    throw std::runtime_error(err);
  }
  return camera_map_.at(serial_number);
}

}  // namespace jet