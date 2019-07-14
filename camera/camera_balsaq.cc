//%bin(camera_balsaq_main)
#include "camera/camera_balsaq.hh"
#include "camera/camera_manager.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/time/duration.hh"
#include "infrastructure/time/time_utils.hh"

#include <cstddef>
#include <iostream>
#include <sstream>
#include <thread>

namespace jet {

void CameraBq::init(const Config& config) {
  // 20Hz Update Rate
  loop_delay_microseconds = 50000;
  assert(config["use_shmem"]);
  camera_ = camera_manager_.get_camera(config["serial_number"].as<std::string>());
  std::cout << "Camera BQ: camera serial " << config["serial_number"].as<std::string>() << std::endl;
  cap_ = cv::VideoCapture(camera_.v4l_path);
  camera_config_ = generate_capture_config(config);
  initialize_camera_hardware(camera_config_, cap_);

  if (config["use_shmem"]) {
    camera_shmem_publisher_ = std::make_unique<jet::SharedStructPublisher>("camera_image_channel");
  } else {
    publisher_ = make_publisher("camera_image_channel");
  }
}

void CameraBq::loop() {
  cv::Mat camera_frame;
  std::cout << "Camera BQ: trying to get a frame" << std::endl;

  if (cap_.read(camera_frame)) {
    // Get the image capture timestamp, which is the time that the first byte was captured, as returned by
    // clock_gettime().
    const long int cap_time_msec = cap_.get(cv::CAP_PROP_POS_MSEC);
    const Timestamp cap_time_vehicle = time::msec_monotonic_to_vehicle_monotonic(cap_time_msec);
    gonogo().go();
    last_msg_recvd_timestamp_ = cap_time_vehicle;
    CameraImageMessage message = create_camera_image_message(camera_frame, camera_.serial_number, cap_time_vehicle);

    if (publisher_) {
      publisher_->publish(message);
    }
    if (camera_shmem_publisher_) {
      camera_shmem_publisher_->publish(message);
    }
  } else {
  }
  if (last_msg_recvd_timestamp_ < get_current_time() - Duration::from_seconds(1)) {
    gonogo().nogo("More than 1 second since last camera frame");
  }
}
void CameraBq::shutdown() {
  std::cout << "Camera BQ shutting down." << std::endl;
}

}  // namespace jet

BALSA_QUEUE_MAIN_FUNCTION(jet::CameraBq)
