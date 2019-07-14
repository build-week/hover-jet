//%bin(fiducial_detection_balsaq_main)
//%deps(balsa_queue)

#include "vision/fiducial_detection_balsaq.hh"
#include "camera/camera_image_message.hh"
#include "config/fiducial_map/read_fiducial_map.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/time/duration.hh"

#include <iostream>

namespace jet {

void FiducialDetectionBq::init(const Config& config) {
  // 20Hz Update Rate
  loop_delay_microseconds = 50000;
  assert(config["use_shmem"]);
  if (config["use_shmem"].as<bool>()) {
    camera_shmem_subscriber_ = std::make_unique<SharedStructSubscriber>("camera_image_channel");
  } else {
    subscriber_ = make_subscriber("camera_image_channel");
  }

  publisher_ = make_publisher("fiducial_detection_channel");
}

void FiducialDetectionBq::loop() {
  CameraImageMessage image_message;

  // Wait until we have the latest image_message
  bool got_msg = false;
  if (subscriber_) {
    // Wait until we have the latest image_message
    while (subscriber_->read(image_message, 1)) {
      got_msg = true;
    }
  } else {
    while (camera_shmem_subscriber_->read(image_message, 0)) {
      got_msg = true;
    }
  }

  if (got_msg) {
    gonogo().go();
    last_msg_recvd_timestamp_ = get_current_time();
    const Calibration camera_calibration = camera_manager_.get_calibration(image_message.camera_serial_number);
    const cv::Mat camera_frame = get_image_mat(image_message);
    std::optional<FiducialDetectionMessage> detection_message =
        create_fiducial_detection_message(camera_frame, camera_calibration, image_message.header.timestamp_ns);
    if (detection_message) {
      publisher_->publish(*detection_message);
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
      cv::waitKey(1);
    }
  }
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last image message");
  }
}

void FiducialDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::FiducialDetectionBq)
