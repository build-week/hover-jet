
#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "camera/camera_manager.hh"

//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace jet {

// A C++ form of the yaml config
//
// Stored this way to avoid parsing overhead when re-reading the
// config each frame
//
// The fields are left intentionally uninitialized, so that
// their use before initialization can be detected with Valgrind
struct CameraConfiguration {
  int frames_per_second;
  double exposure;

  int auto_focus;
  double auto_exposure;

  int width_pixels;
  int height_pixels;
};

class CameraBq : public BalsaQ {
 public:
  CameraBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  cv::VideoCapture cap;

  std::string camera_serial_number_;
  CameraConfiguration camera_config_;
  CameraManager camera_manager_ = CameraManager();
};

}  // namespace jet
