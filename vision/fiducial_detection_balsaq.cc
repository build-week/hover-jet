//%bin(fiducial_detection_balsaq_main)
//%deps(balsa_queue)
//%deps(message)

#include "vision/fiducial_detection_balsaq.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "vision/fiducial_detection_and_pose.hh"

#include <iostream>

namespace jet {

FidicualDetectionBq::FidicualDetectionBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void FidicualDetectionBq::init(int argc, char *argv[]) {
  subscriber_ = make_subscriber("camera_image_channel");
  publisher_ = make_publisher("fiducial_detection_channel");
}

void FidicualDetectionBq::loop() {
  CameraImageMessage message;
  if (subscriber_->read(message, 1)) {
    const cv::Mat camera_frame = get_image_mat(message);
    std::optional<SE3> board_from_camera = detect_board(camera_frame);
    if(board_from_camera){
      // publish a message using *board_from_camera
    }
    // The third and fourth parameters are the marker length and the marker separation respectively.
    // They can be provided in any unit, having in mind that the estimated pose for this board
    // will be measured in the same units (in general, meters are used).
    cv::Mat board_image;
    get_aruco_board()->draw(cv::Size(900, 900), board_image, 50, 1);
    if (OPEN_DEBUG_WINDOWS) {
      cv::imshow("window2", board_image);
      cv::waitKey(2);

      cv::imshow("window", camera_frame);
      cv::waitKey(1);
    }
  }
}

void FidicualDetectionBq::shutdown() {
  std::cout << "fiducal detection BQ shutting down." << std::endl;
}

}  // namespace jet
BALSA_QUEUE_MAIN_FUNCTION(jet::FidicualDetectionBq)
