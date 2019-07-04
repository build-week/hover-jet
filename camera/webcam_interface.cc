#include "camera/webcam_interface.hh"

namespace jet {

// Generate a config structure from a yaml config
//
// Asserts that the required config fields are actually set.
// Has no defaulting behavior.
CameraConfiguration generate_capture_config(const Config& config) {
  assert(config["frames_per_second"]);
  assert(config["exposure"]);
  assert(config["auto_focus"]);
  assert(config["auto_exposure"]);
  assert(config["width_pixels"]);
  assert(config["height_pixels"]);

  CameraConfiguration camera_config;
  {
    camera_config.frames_per_second = config["frames_per_second"].as<int>();
    camera_config.exposure = config["exposure"].as<double>();
    camera_config.auto_focus = config["auto_focus"].as<int>();
    camera_config.auto_exposure = config["auto_exposure"].as<double>();
    camera_config.width_pixels = config["width_pixels"].as<int>();
    camera_config.height_pixels = config["height_pixels"].as<int>();
  }
  return camera_config;
}

void initialize_camera_hardware(const CameraConfiguration& camera_config, cv::VideoCapture& cap) {
  // Set the buffer to a single image so that we are always grabbing the latest frame
  cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_config.width_pixels);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_config.height_pixels);
  cap.set(cv::CAP_PROP_FPS, camera_config.frames_per_second);
  cap.set(cv::CAP_PROP_AUTOFOCUS, camera_config.auto_focus);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, camera_config.auto_exposure);

  // Must grab a frame to fully turn on the camera
  cap.grab();

  // After the camera is turned on, we set the exposure.
  cap.set(cv::CAP_PROP_EXPOSURE, camera_config.exposure);
}

}  // namespace jet