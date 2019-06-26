//%deps(yaml-cpp)
//%deps(-lstdc++fs)

#include "camera/camera_manager.hh"

namespace fs = std::experimental::filesystem;

namespace jet {

CameraManager::CameraManager() {
  load_configs();
  if (camera_map_.size() == 0) {
    std::string err = std::string("No camera configs read!");
    throw std::runtime_error(err);
  }
}

void CameraManager::load_configs() {
  // directory iteration from https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c/37494654#37494654
  const std::string config_dir = "/jet/camera/cfg";
  for (const auto & entry : fs::directory_iterator(config_dir)) {
    const std::string filename = entry.path();
    if (filename == "." || filename == ".."){ 
      continue;
    }
    std::optional<YAML::Node> config = read_YAML(filename);
    std::optional<Camera> camera = parse_config(*config);
    if (camera.has_value()) {
      camera_map_[camera->serial_number] = *camera;
    }
  }
}

std::optional<YAML::Node> CameraManager::read_YAML(const std::string& filepath) const {
  try {
    return YAML::LoadFile(filepath);
  } catch (YAML::BadFile e) {
    const std::string err = std::string("Could not find YAML file ") + filepath;
    throw std::runtime_error(err);
  }
}

std::optional<Camera> CameraManager::parse_config(const YAML::Node& cfg) const {
  Camera camera;
  // path & video index
  std::cout << "  " << std::endl;
  camera.serial_number = cfg["serial_number"].as<std::string>();
  camera.v4l_path = "/dev/v4l/by-id/" + cfg["v4l_path"].as<std::string>();
  const auto optional_camera_index = follow_v4l_path(camera.v4l_path);
  
  if (!optional_camera_index.has_value()){
    return std::nullopt;
  }
  else{
    camera.video_index = optional_camera_index.value();
  }
  // calibration values
  std::vector<double> camera_matrix_values = cfg["camera_matrix"].as<std::vector<double>>();
  camera.calibration.camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_values.data()).clone();
  std::vector<double> distortion_coefficient_values = cfg["distortion_coefficients"].as<std::vector<double>>();
  camera.calibration.distortion_coefficients = cv::Mat(1, 5, CV_64F, distortion_coefficient_values.data()).clone();
  return camera;
}

std::optional<int> CameraManager::follow_v4l_path(const std::string& path) const {
  std::error_code ec;
  const std::string v4l_path = fs::canonical(path, ec).string();
  if (ec) {
    return std::nullopt;
  } 
  // note that this will break for >10 cameras
  // TODO better way
  const int index = v4l_path.back() - '0';
  return index;
}

Camera CameraManager::get_camera(const std::string& serial_number) const {
  if (camera_map_.count(serial_number) == 0) {
    const std::string err = std::string(
        "The camera config passed to start the camera BQ specified "
        "a serial number that doesn't match the camera we have plugged in.");
    throw std::runtime_error(err);
  }
  return camera_map_.at(serial_number);
}

}  // namespace jet