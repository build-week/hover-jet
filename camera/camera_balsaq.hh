
#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include "camera/camera_image_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace jet {

class CameraBq : public BalsaQ {
 public:
  CameraBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  static const int CAMERA_FPS = 10;
  PublisherPtr publisher_;
  cv::VideoCapture cap;
  std::string camera_serial_number_;
};

}  // namespace jet
