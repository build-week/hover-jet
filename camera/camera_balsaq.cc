//%bin(camera_balsaq_main)
#include "camera/camera_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/time/duration.hh"
#include "camera/camera_manager.hh"

#include <cstddef>
#include <iostream>
#include <sstream>

namespace jet {

namespace {

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
}  // namespace

void CameraBq::init(const Config& config) {
  const Camera camera = camera_manager_.get_camera(config["serial_number"].as<std::string>());
  camera_serial_number_ = camera.serial_number;
  std::cout << "Camera BQ: camera serial " << config["serial_number"].as<std::string>() << std::endl;
  cap = cv::VideoCapture(camera.video_index);

  camera_config_ = generate_capture_config(config);

  cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_config_.width_pixels);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_config_.height_pixels);
  cap.set(cv::CAP_PROP_FPS, camera_config_.frames_per_second);

  publisher_ = make_publisher("camera_image_channel");
}

void CameraBq::loop() {
  cv::Mat camera_frame;
  std::cout << "Camera BQ: trying to get a frame" << std::endl;

  cap.set(cv::CAP_PROP_AUTOFOCUS, camera_config_.auto_focus);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, camera_config_.auto_exposure);
  cap.set(cv::CAP_PROP_EXPOSURE, camera_config_.exposure);

  const auto current_time = get_current_time();
  if (cap.read(camera_frame)) {
    gonogo().go();
    last_msg_recvd_timestamp_ = get_current_time();
    CameraImageMessage message;
    const std::size_t n_elements = camera_frame.rows * camera_frame.cols * 3u;
    message.image_data.resize(n_elements);
    constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
    if (camera_frame.isContinuous()) {
      std::memcpy(message.image_data.data(), camera_frame.data, SIZE_OF_UCHAR * n_elements);
    }
    message.timestamp = current_time;
    message.height = camera_frame.size().height;
    message.width = camera_frame.size().width;
    message.camera_serial_number = camera_serial_number_;
    publisher_->publish(message);
    std::cout << "Camera BQ: publishes a camera frame " << message.width << " " << message.height << std::endl;
  }
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last camera frame");
  }
}
void CameraBq::shutdown() {
  std::cout << "Camera BQ shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraBq)
