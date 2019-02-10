#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"
#include "vision/fiducial_detection_and_pose.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "camera/camera_manager.hh"

namespace jet {

class FidicualDetectionBq : public BalsaQ {
 public:
  FidicualDetectionBq() = default;
  void init(int argc, char *argv[]) override;
  void loop() override;
  void shutdown() override;

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
  static constexpr bool OPEN_DEBUG_WINDOWS = false;
  CameraManager camera_manager_ = CameraManager();
};

}  // namespace jet
