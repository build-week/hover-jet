#include "camera/camera_balsaq.hh"

// #include "infrastructure/comms/schemas/nop_structure_example.hh"

#include <iostream>
#include <sstream>
#include <cstddef>
#include <chrono>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

CameraBq::CameraBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void CameraBq::init() {
  std::cout << "INIT!" << std::endl;
  cap = cv::VideoCapture(0);
  // 0 is the id of video device.0 if you have only one camera.
  publisher_ = make_publisher("camera_image_channel");
}

void CameraBq::loop() {
  
  cv::Mat cameraFrame;
  if (cap.read(cameraFrame)) {
    CameraImageMessage message;
    const std::size_t n_elements = cameraFrame.rows * cameraFrame.cols * 3u;
    message.image_data.resize(n_elements);
    constexpr std::size_t SIZE_OF_UCHAR = sizeof(uint8_t);
    if (cameraFrame.isContinuous()){
      std::memcpy(message.image_data.data(), cameraFrame.data, SIZE_OF_UCHAR*n_elements);
    }
    message.timestamp_ns = 0;//std::chrono::high_resolution_clock::now();
    message.height = cameraFrame.size().height;
    message.width = cameraFrame.size().width;
    publisher_->publish(message);
  } else {
    std::cout << "camera grab failed" << std::endl;
  }
}
void CameraBq::shutdown() { std::cout << "Shutting down!" << std::endl; }

}  // namespace jet