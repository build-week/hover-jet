//%bin(camera_viewer_main)
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "camera/camera_viewer_bq.hh"

namespace jet {

void CameraViewerBq::init(const Config& config) {
  subscriber_ = make_subscriber("camera_image_channel");
}

void CameraViewerBq::loop() {
  CameraImageMessage image_message;

  // Wait until we have the latest image_message
  bool got_msg = false;
  while (subscriber_->read(image_message, 1)) {
    got_msg = true;
  }

  if (got_msg) {
    gonogo().go();
    const cv::Mat camera_frame = get_image_mat(image_message);

    cv::imshow("camera image", camera_frame);
    cv::waitKey(1);
  }
}
void CameraViewerBq::shutdown() {
  std::cout << "Camera viewer shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraViewerBq)
 