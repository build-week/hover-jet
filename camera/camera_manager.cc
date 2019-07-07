//%deps(yaml-cpp)
//%deps(stdc++fs)

#include "camera/camera_manager.hh"

namespace fs = std::experimental::filesystem;

namespace jet {
namespace {

std::string config_search_location() {
  return "/jet/camera/cfg";
}

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
  if (camera_calibration_from_serial_.size() == 0) {
    std::string err = std::string("No calibrations read!");
    throw std::runtime_error(err);
  }
}

void CameraManager::load_configs() {
  // directory iteration from
  // https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c/37494654#37494654
  const std::string config_dir = config_search_location();
  for (const auto& entry : fs::directory_iterator(config_dir)) {
    const std::string filename = entry.path();
    if (filename == "." || filename == "..") {
      continue;
    }

    const std::optional<YAML::Node> config = read_YAML(filename);
    if (!config) {
      continue;
    }

    const std::optional<Camera> camera = extract_v4l_path(*config);
    if (camera.has_value()) {
      camera_from_serial_[camera->serial_number] = *camera;
    }

    const auto serial_number = (*config)["serial_number"].as<std::string>();
    const auto calibration = extract_calibration(*config);
    camera_calibration_from_serial_[serial_number] = calibration;
  }
}

Calibration CameraManager::get_calibration(const std::string& serial_number) const {
  if (camera_calibration_from_serial_.count(serial_number) == 0) {
    const std::string err = std::string("Could not find a camera serial number in: ") + config_search_location();
    throw std::runtime_error(err);
  }
  const auto calibration_from_map = camera_calibration_from_serial_.at(serial_number);
  Calibration result;
  result.camera_matrix = calibration_from_map.camera_matrix.clone();
  result.distortion_coefficients = calibration_from_map.distortion_coefficients.clone();
  return result;
}

Camera CameraManager::get_camera(const std::string& serial_number) const {
  if (camera_from_serial_.count(serial_number) == 0) {
    std::stringstream ss;
    ss << "The camera we are looking for (" << serial_number << ") is not present in /dev/v4l";
    throw std::runtime_error(ss.str());
  }
  return camera_from_serial_.at(serial_number);
}
}  // namespace jet