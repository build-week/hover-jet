#include "camera/camera_balsaq.hh"
#include "vision/fiducial_detection_and_pose.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
// #include "infrastructure/comms/schemas/nop_structure_example.hh"

#include <iostream>
#include <sstream>

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
  publisher_ = make_publisher("pub_channel_name");
}

void CameraBq::loop() {
  
  cv::Mat cameraFrame;
  if (cap.read(cameraFrame)) {
    std::vector<vision::MarkerDetection> marker_detections =
        vision::detect_markers(cameraFrame);
    std::cout << "number of detections " << marker_detections.size()
              << std::endl;
    for (auto const& detection : marker_detections) {
      std::cout << "detected artag #" << detection.id << std::endl;
    }
    publisher_->publish_raw("Camera balsa queue looping");
  } else {
    std::cout << "camera grab failed" << std::endl;
  }
}
void CameraBq::shutdown() { std::cout << "Shutting down!" << std::endl; }

}  // namespace jet