//%bin(integrated_camera_vision_balsaq_main)
#include "camera/integrated_camera_vision_balsaq.hh"
#include "camera/camera_manager.hh"
#include "config/fiducial_map/read_fiducial_map.hh"
#include "infrastructure/balsa_queue/bq_main_macro.hh"
#include "infrastructure/time/duration.hh"
#include "infrastructure/time/time_utils.hh"
#include "vision/fiducial_detection_and_pose.hh"
#include "vision/fiducial_detection_message.hh"

#include <cstddef>
#include <iostream>
#include <sstream>
#include <thread>

namespace jet {

void CameraBq::init(const Config& config) {
  camera_ = camera_manager_.get_camera(config["serial_number"].as<std::string>());
  std::cout << "Camera BQ: camera serial " << config["serial_number"].as<std::string>() << std::endl;
  cap_ = cv::VideoCapture(camera_.v4l_path);
  camera_config_ = generate_capture_config(config);
  initialize_camera_hardware(camera_config_, cap_);
  image_publisher_ = make_publisher("camera_image_channel");
  
  fiducial_detection_publisher_ = make_publisher("fiducial_detection_channel");

  
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

    image_publisher_->publish(message);
    std::cout << "Camera BQ: publishes a camera frame " << message.width << " " << message.height << std::endl;

    const Calibration camera_calibration = camera_manager_.get_calibration(message.camera_serial_number);
    const cv::Mat camera_frame = get_image_mat(message);
    std::optional<FiducialDetectionMessage> detection_message =
        create_fiducial_detection_message(camera_frame, camera_calibration, message.header.timestamp_ns);
    if (detection_message) {
      fiducial_detection_publisher_->publish(*detection_message);
    }
    else{
      std::cout << "no fiducial detected";
    }
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
