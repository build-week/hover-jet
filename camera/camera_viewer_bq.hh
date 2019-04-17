
#pragma once

#include <infrastructure/balsa_queue/balsa_queue.hh>
#include "camera/camera_image_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"

//%deps(opencv)
#include <opencv2/opencv.hpp>

namespace jet {

class CameraViewerBq : public BalsaQ {
 public:
  CameraViewerBq() = default;
  void init(const Config& config);
  void loop();
  void shutdown();

 private:
  SubscriberPtr subscriber_;
 	
};

}  // namespace jet
