
#pragma once

#include "infrastructure/balsa_queue/balsa_queue.hh"
#include "camera/camera_image_message.hh"
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/comms/shared_struct/shared_struct_subscriber.hh"

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
  SubscriberPtr camera_subscriber_;
  SubscriberPtr fiducial_subscriber_;
  std::unique_ptr<SharedStructSubscriber> camera_shmem_subscriber_;
};

}  // namespace jet
