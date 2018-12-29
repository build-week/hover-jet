#include "vision/fiducial_pose.hh"
#include "camera/camera_balsaq.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
// #include "infrastructure/comms/schemas/nop_structure_example.hh"

#include <iostream>
#include <sstream>

//%deps(balsa_queue)
//%deps(message)

namespace jet {

CameraBq::CameraBq() {
  set_comms_factory(std::make_unique<MqttCommsFactory>());
}

void CameraBq::init() {
  std::cout << "INIT!" << std::endl;
  cv::VideoCapture cap(0);

  publisher_ = make_publisher("pub_channel_name");
}

void CameraBq::loop() {
  cv::VideoCapture stream1(0);
      // 0 is the id of video device.0 if you have only one camera.
  cv::Mat cameraFrame;
  stream1.read(cameraFrame);
  cv::imshow("cam", cameraFrame);
  vision::detect_markers(cameraFrame);
  publisher_->publish_raw("looping!");
}
void CameraBq::shutdown() {
  std::cout << "Shutting down!" << std::endl;
}

}  // namespace jet
