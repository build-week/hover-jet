//%bin(camera_viewer_main)
#include "camera/camera_viewer_bq.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "vision/fiducial_detection_message.hh"

namespace jet {

void CameraViewerBq::init(const Config& config) {
  fiducial_subscriber_ = make_subscriber("fiducial_detection_channel");
  assert(config["use_shmem"]);

  if (config["use_shmem"].as<bool>()) {
    camera_shmem_subscriber_ = std::make_unique<SharedStructSubscriber>("camera_image_channel");
  } else {
    camera_subscriber_ = make_subscriber("camera_image_channel");
  }
}

void CameraViewerBq::loop() {
  CameraImageMessage image_message;
  FiducialDetectionMessage fiducial_message;

  // Wait until we have the latest image_message
  bool got_msg = false;
  if (camera_subscriber_)
  {
    // Wait until we have the latest image_message
    while (camera_subscriber_->read(image_message, 1)) {
      got_msg = true;
    }
  } else {
    while (camera_shmem_subscriber_->read(image_message, 0)) {
      got_msg = true;
    }
  }

  if (got_msg) {
    got_msg = false;
    while (fiducial_subscriber_->read(fiducial_message, 1)) {
      got_msg = true;
    }
    const cv::Mat camera_frame = get_image_mat(image_message);
    if (got_msg) {
      gonogo().go();

      for (auto& point_association : fiducial_message.board_points_image_points) {
        const auto image_point = point_association.point_image_space;
        cv::circle(camera_frame, cv::Point(image_point.vals[0],
          image_point.vals[1]), 10, cv::Scalar( 128, 128, 1 ));
        std::cout << image_point.vals[0] << " " << image_point.vals[1] << std::endl;
      }
    }

      cv::imshow("camera image", camera_frame);
      cv::waitKey(1);
  }
}
void CameraViewerBq::shutdown() {
  std::cout << "Camera viewer shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraViewerBq)
