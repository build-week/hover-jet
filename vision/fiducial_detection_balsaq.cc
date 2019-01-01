
//%deps(balsa_queue)
//%deps(message)

#include "vision/fiducial_detection_balsaq.hh"
#include "camera/camera_image_message.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/schemas/demo_message.hh"

#include <iostream>

namespace jet {

FidicualDetectionBq::FidicualDetectionBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void FidicualDetectionBq::init() {
  subscriber_ = make_subscriber("camera_image_channel");
  publisher_ = make_publisher("fiducial_detection_channel");

}

void FidicualDetectionBq::loop() {
  std::cout << "before read attempt" << std::endl;
  CameraImageMessage message;
  if (subscriber_->read(message, 1)) {
    std::cout << "At " << message.header.timestamp_ns
              << " FidicualDetectionBq received message #: "
              << std::endl;
      cv::Mat cameraFrame = getImageMat(message);
      std::vector<MarkerDetection> marker_detections =
          detect_markers(cameraFrame);
      std::cout << "number of detections " << marker_detections.size()
                << std::endl;
      for (auto const& detection : marker_detections) {
        std::cout << "detected artag #" << detection.id << std::endl;
      }    
  }
  std::cout << "after read attempt" << std::endl;
}

void FidicualDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
