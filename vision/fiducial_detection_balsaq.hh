#include <infrastructure/balsa_queue/balsa_queue.hh>
//%deps(opencv)
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "camera/camera_image_message.hh"
#include "vision/fiducial_detection_and_pose.hh"

#include "infrastructure/comms/mqtt_comms_factory.hh"


namespace jet {

class FidicualDetectionBq : public BalsaQ {
 public:
  FidicualDetectionBq();
  void init();
  void loop();
  void shutdown();

 private:
  PublisherPtr publisher_;
  SubscriberPtr subscriber_;
  
};

}  // namespace jet
