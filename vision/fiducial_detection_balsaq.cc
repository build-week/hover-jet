//%bin(fiducial_detection_balsaq_main)
//%deps(balsa_queue)
//%deps(message)

#include "vision/fiducial_detection_balsaq.hh"
#include "vision/fiducial_detection_and_pose.hh"
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
    const cv::Mat camera_frame = get_image_mat(message);
    const std::vector<MarkerDetection> marker_detections =
        detect_markers(camera_frame);

    for (const auto & detection : marker_detections) {
      std::cout << detection.marker_center_from_camera.translation().norm() << std::endl;
    }
  }
}

void FidicualDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::FidicualDetectionBq)
