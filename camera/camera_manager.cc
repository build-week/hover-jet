//%deps(yaml-cpp)
//%deps(stdc++fs)

#include "camera/camera_manager.hh"

namespace fs = std::experimental::filesystem;

namespace jet {
namespace {

std::optional<YAML::Node> read_YAML(const std::string& filepath) {
  try {
    return YAML::LoadFile(filepath);
  } catch (YAML::BadFile e) {
    const std::string err = std::string("Could not find YAML file ") + filepath;
    throw std::runtime_error(err);
  }
}

Calibration extract_calibration(const YAML::Node& cfg) {
  Calibration calibration;
  std::vector<double> camera_matrix_values = cfg["camera_matrix"].as<std::vector<double>>();
  calibration.camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_values.data()).clone();
  std::vector<double> distortion_coefficient_values = cfg["distortion_coefficients"].as<std::vector<double>>();
  calibration.distortion_coefficients = cv::Mat(1, 5, CV_64F, distortion_coefficient_values.data()).clone();
  return calibration;
}

std::optional<Camera> extract_v4l_path(const YAML::Node& cfg) {
  Camera camera;
  // path & video index
  camera.serial_number = cfg["serial_number"].as<std::string>();
  camera.v4l_path = "/dev/v4l/by-id/" + cfg["v4l_path"].as<std::string>();

  if (fs::exists(camera.v4l_path)) {
    return camera;
  } else {
    return {};
  }
}

}  // namespace

CameraManager::CameraManager() {
  load_configs();
  if (calibration_map_.size() == 0) {
    std::string err = std::string("No calibrations read!");
    throw std::runtime_error(err);
  }
}

void CameraManager::load_configs() {
  // directory iteration from
  // https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c/37494654#37494654
  const std::string config_dir = "/jet/camera/cfg";
  for (const auto& entry : fs::directory_iterator(config_dir)) {
    const std::string filename = entry.path();
    if (filename == "." || filename == "..") {
      continue;
    }

    const std::optional<YAML::Node> config = read_YAML(filename);
    if (!config) {
      throw std::runtime_error("Could not open config");
    }

    const std::optional<Camera> camera = extract_v4l_path(*config);
    if (camera.has_value()) {
      camera_map_[camera->serial_number] = *camera;
    }

    const auto calibration = extract_calibration(*config);
    calibration_map_[camera->serial_number] = calibration;
  }
}

Calibration CameraManager::get_calibration(const std::string& serial_number) const {
  if (calibration_map_.count(serial_number) == 0) {
    const std::string err = std::string("Camera serial number is not known.");
    throw std::runtime_error(err);
  }
  return calibration_map_.at(serial_number);
}

Camera CameraManager::get_camera(const std::string& serial_number) const {
  if (camera_map_.count(serial_number) == 0) {
    const std::string err = std::string("Camera is not plugged in");
    throw std::runtime_error(err);
  }
  return camera_map_.at(serial_number);
}

}  // namespace jet