//%bin(camera_balsaq_main)
#include "camera/camera_balsaq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

constexpr double WEBCAM_EXPOSURE = 0.01;

void CameraBq::init(int argc, char *argv[]) {
  cap = cv::VideoCapture(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  // 0 is the id of video device.0 if you have only one camera.
  publisher_ = make_publisher("camera_image_channel");
}

void CameraBq::loop() {
  cv::Mat camera_frame;
  std::cout << "Camera BQ: trying to get a frame" << std::endl;

  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
  cap.set(cv::CAP_PROP_EXPOSURE, WEBCAM_EXPOSURE);

  if (cap.read(camera_frame)) {
    CameraImageMessage message;
    const std::size_t n_elements = camera_frame.rows * camera_frame.cols * 3u;
    message.image_data.resize(n_elements);
    constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
    if (camera_frame.isContinuous()) {
      std::memcpy(message.image_data.data(), camera_frame.data,
                  SIZE_OF_UCHAR * n_elements);
    }
    message.timestamp = get_current_time();
    message.height = camera_frame.size().height;
    message.width = camera_frame.size().width;
    publisher_->publish(message);
    std::cout << "CAMERA TASK: publishes a camera frame " << message.width << " "
              << message.height << std::endl;
  } else {
  }
}
void CameraBq::shutdown() {
  std::cout << "Camera process shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraBq)
