//%bin(camera_balsaq_main)
#include "camera/camera_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/time/duration.hh"
#include "camera/camera_manager.hh"

#include <cstddef>
#include <iostream>
#include <sstream>
#include <thread>

namespace jet {

namespace {

// Generate a config structure from a yaml config
//
// Asserts that the required config fields are actually set.
// Has no defaulting behavior.
CameraConfiguration generate_capture_config(const Config& config) {
  assert(config["frames_per_second"]);
  assert(config["webcam_exposure"]);
  assert(config["auto_focus"]);
  assert(config["auto_exposure"]);
  assert(config["width_pixels"]);
  assert(config["height_pixels"]);

  CameraConfiguration camera_config;
  {
    camera_config.frames_per_second = config["frames_per_second"].as<int>();
    camera_config.exposure = config["webcam_exposure"].as<int>();
    camera_config.auto_focus = config["auto_focus"].as<int>();
    camera_config.auto_exposure = config["auto_exposure"].as<int>();
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
  cap_ = cv::VideoCapture(camera.v4l_path);

  camera_config_ = generate_capture_config(config);

  // Set the buffer to a single image so that we are always grabbing the latest frame
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_config_.width_pixels);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_config_.height_pixels);
  cap_.set(cv::CAP_PROP_FPS, camera_config_.frames_per_second);
  cap_.set(cv::CAP_PROP_AUTOFOCUS, camera_config_.auto_focus);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, camera_config_.auto_exposure);

  // Must grab a frame to fully turn on the camera
  cap_.grab();

  // After the camera is turned on, we set the exposure.
  cap_.set(cv::CAP_PROP_EXPOSURE, camera_config_.exposure);

  publisher_ = make_publisher("camera_image_channel");
}

void CameraBq::loop() {
  cv::Mat camera_frame;
  std::cout << "Camera BQ: trying to get a frame" << std::endl;

  if (cap_.read(camera_frame)) {
    // Get the image capture timestamp, which is the time that the first byte was captured, as returned by clock_gettime(). 
    const long int cap_time_msec = cap_.get(cv::CAP_PROP_POS_MSEC);
    const Timestamp cap_time_vehicle = msec_monotonic_to_vehicle_monotonic(cap_time_msec);
    gonogo().go();
    last_msg_recvd_timestamp_ = cap_time_vehicle;

    // Pack a message
    CameraImageMessage message;
    const std::size_t n_elements = camera_frame.rows * camera_frame.cols * 3u;
    message.image_data.resize(n_elements);
    constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
    if (camera_frame.isContinuous()) {
      std::memcpy(message.image_data.data(), camera_frame.data, SIZE_OF_UCHAR * n_elements);
    }
    message.timestamp = cap_time_vehicle;
    message.height = camera_frame.size().height;
    message.width = camera_frame.size().width;
    message.camera_serial_number = camera_serial_number_;
    publisher_->publish(message);
    std::cout << "Camera BQ: publishes a camera frame " << message.width << " " << message.height << std::endl;
  } else {
  }
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last camera frame");
  }
}
void CameraBq::shutdown() {
  std::cout << "Camera BQ shutting down." << std::endl;
}

Timestamp CameraBq::msec_monotonic_to_vehicle_monotonic(long int cap_time_msec) const {
  // Get the reported monotonic time in milliseconds
  const std::chrono::milliseconds cap_time_monotonic_msec(cap_time_msec);
  // Convert it to a timepoint
  const std::chrono::time_point<std::chrono::steady_clock> cap_time_monotonic(cap_time_monotonic_msec);

  // Get the current time in monotonic
  const auto cur_time_monotonic = std::chrono::steady_clock::now();
  // Get the current time in the vehicle wall clock
  const auto cur_time_vehicle = get_current_time().time_point();
  const auto cur_time_from_cap_time = cur_time_monotonic - cap_time_monotonic;
  const auto cap_time_vehicle = cur_time_vehicle - cur_time_from_cap_time;

  const auto wall_from_monotonic = (cur_time_vehicle.time_since_epoch()) - (cur_time_monotonic.time_since_epoch());

  return Timestamp(cap_time_vehicle);
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraBq)
