//%bin(camera_balsaq_main)
#include "camera/camera_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/time/duration.hh"
// #include "camera/calibration_manager.hh"
#include "camera/camera_manager.hh"

#include <cstddef>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

constexpr double WEBCAM_EXPOSURE = 0.01;

void CameraBq::init(const Config& config) {
  Camera camera;
  std::cout << "Camera BQ: camera serial " << config["serial_number"].as<std::string>() << std::endl;
  camera = camera_manager_.get_camera(config["serial_number"].as<std::string>());
  camera_serial_number_ = camera.serial_number;
  cap = cv::VideoCapture(camera.video_index);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
  publisher_ = make_publisher("camera_image_channel");
}

void CameraBq::loop() {
  cv::Mat camera_frame;
  std::cout << "Camera BQ: trying to get a frame" << std::endl;

  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
  cap.set(cv::CAP_PROP_EXPOSURE, WEBCAM_EXPOSURE);

  const auto current_time = get_current_time();
  if (cap.read(camera_frame)) {
    gonogo_.go();
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
    gonogo_.nogo("More than 1 second since last camera frame");
  }
}
void CameraBq::shutdown() {
  std::cout << "Camera BQ shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraBq)
