//%bin(fiducial_detection_balsaq_main)
//%deps(balsa_queue)

#include "vision/fiducial_detection_balsaq.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "vision/fiducial_detection_and_pose.hh"
#include "vision/fiducial_detection_message.hh"

#include <iostream>

namespace jet {

void FidicualDetectionBq::init(int argc, char *argv[]) {
  subscriber_ = make_subscriber("camera_image_channel");
  publisher_ = make_publisher("fiducial_detection_channel");
}

void FidicualDetectionBq::loop() {
  CameraImageMessage image_message;

  // Wait until we have the latest image_message
  bool got_msg = false;
  while (subscriber_->read(image_message, 1)) {
    got_msg = true;
  }

  if (got_msg) {
    const cv::Mat camera_frame = get_image_mat(image_message);
    const std::optional<SE3> board_from_camera = detect_board(camera_frame);
    if (board_from_camera) {
      // publish a fiducial message using *board_from_camera
      FiducialDetectionMessage detection_message;
      const jcc::Vec6 log_fiducial_from_camera = board_from_camera->log();
      detection_message.fiducial_from_camera_log[0] = log_fiducial_from_camera[0];
      detection_message.fiducial_from_camera_log[1] = log_fiducial_from_camera[1];
      detection_message.fiducial_from_camera_log[2] = log_fiducial_from_camera[2];
      detection_message.fiducial_from_camera_log[3] = log_fiducial_from_camera[3];
      detection_message.fiducial_from_camera_log[4] = log_fiducial_from_camera[4];
      detection_message.fiducial_from_camera_log[5] = log_fiducial_from_camera[5];

      detection_message.timestamp = image_message.timestamp;

      publisher_->publish(detection_message);
      // reconstruct with eg
      // board_from_camera = SE3::exp(Eigen::Map<jcc::Vec6>>(array));

      std::cout << "detection: " << board_from_camera->translation().transpose()
                << std::endl;
    }
    // The third and fourth parameters are the marker length and the marker separation
    // respectively. They can be provided in any unit, having in mind that the estimated
    // pose for this board will be measured in the same units (in general, meters are
    // used).
    if (OPEN_DEBUG_WINDOWS) {
      cv::Mat board_image;
      get_aruco_board()->draw(cv::Size(900, 900), board_image, 50, 1);
      cv::imshow("board image", board_image);
      cv::waitKey(1);

      cv::imshow("camera image", camera_frame);
      cv::waitKey(2);
    }
  }
}

void FidicualDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::FidicualDetectionBq)
