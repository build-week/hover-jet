
//%deps(balsa_queue)
//%deps(message)

#include "vision/fiducial_detection_balsaq.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

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
  CameraImageMessage message;
  if (subscriber_->read(message, 1)) {
    std::cout << "At " << message.header.timestamp_ns
              << " FidicualDetectionBq received message #: " << std::endl;
    const cv::Mat camera_frame = get_image_mat(message);
    const std::vector<MarkerDetection> marker_detections =
        detect_markers(camera_frame);
    std::cout << "number of detections " << marker_detections.size()
              << std::endl;
    cv::imshow("window", camera_frame);
    cv::waitKey(1); // to get window to persist
    for (const auto & detection : marker_detections) {
      std::cout << "detected artag #" << detection.id << std::endl;
      std::cout << detection.marker_center_from_camera.translation().transpose()
                << std::endl;
    }
  }
}

void FidicualDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::FidicualDetectionBq)
